use std::env;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PinocchioLinkMode {
    Static,
    Dynamic,
}

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("missing manifest dir"));
    let dependency_prefix =
        find_pinocchio_dependency_prefix(&manifest_dir).expect("failed to locate Pinocchio deps");
    let dependency_lib_dir = find_pinocchio_dependency_lib_dir(&dependency_prefix)
        .expect("failed to locate Pinocchio dependency library dir");
    let urdfdom_headers_include_dir = find_urdfdom_headers_include_dir(&dependency_prefix)
        .expect("failed to locate urdfdom headers include dir");
    let urdfdom_parser_include_dir = find_urdfdom_parser_include_dir(&dependency_prefix)
        .expect("failed to locate urdfdom parser include dir");

    // One directory watch each: Cargo tracks subtree changes without emitting thousands of
    // rerun-if-changed lines (recursive listing was ~1.3k paths and slowed every `cargo build`).
    println!("cargo:rerun-if-env-changed=AIRBOT_PINOCCHIO_DEP_PREFIX");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("ffi").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("third_party/pinocchio").display()
    );
    println!("cargo:rerun-if-changed=src/model/pinocchio_ffi.rs");
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("assets/urdf/play.urdf").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("assets/urdf/play_e2.urdf").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("assets/urdf/play_g2.urdf").display()
    );

    let native_install = build_native(&manifest_dir, &dependency_prefix, &dependency_lib_dir);
    let native_lib_dir = native_install.join("lib");
    let link_mode = detect_pinocchio_link_mode(&native_lib_dir);

    let mut bridge = cxx_build::bridges(["src/model/pinocchio_ffi.rs"]);
    bridge
        .file(manifest_dir.join("ffi/pinocchio_shim.cpp"))
        .include(&manifest_dir)
        .include(native_install.join("include"))
        .include(manifest_dir.join("third_party/pinocchio/include"))
        .include("/usr/include/eigen3")
        .include(&urdfdom_parser_include_dir)
        .include(&urdfdom_headers_include_dir)
        .flag_if_supported("-std=c++17");
    if link_mode == PinocchioLinkMode::Static {
        bridge
            .define("PINOCCHIO_STATIC", None)
            .define("PINOCCHIO_PARSERS_STATIC", None);
    }
    bridge.compile("airbot_pinocchio_ffi");

    println!(
        "cargo:rustc-link-search=native={}",
        native_lib_dir.display()
    );
    println!(
        "cargo:rustc-link-search=native={}",
        dependency_lib_dir.display()
    );
    println!(
        "cargo:rustc-link-lib={}={}",
        link_mode.rustc_link_kind(),
        "pinocchio_parsers"
    );
    println!(
        "cargo:rustc-link-lib={}={}",
        link_mode.rustc_link_kind(),
        "pinocchio_default"
    );
    println!("cargo:rustc-link-lib=dylib=boost_filesystem");
    println!("cargo:rustc-link-lib=dylib=boost_serialization");
    println!("cargo:rustc-link-lib=dylib=urdfdom_sensor");
    println!("cargo:rustc-link-lib=dylib=urdfdom_model_state");
    println!("cargo:rustc-link-lib=dylib=urdfdom_model");
    println!("cargo:rustc-link-lib=dylib=urdfdom_world");
    println!("cargo:rustc-link-lib=dylib=console_bridge");
    println!("cargo:rustc-link-lib=dylib=tinyxml2");

    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux") {
        for scope in [
            "cargo:rustc-link-arg",
            "cargo:rustc-link-arg-tests",
            "cargo:rustc-link-arg-bins",
        ] {
            println!("{scope}=-Wl,-rpath,{}", native_lib_dir.display());
            println!("{scope}=-Wl,-rpath,{}", dependency_lib_dir.display());
        }
    }
}

fn build_native(
    manifest_dir: &Path,
    dependency_prefix: &Path,
    dependency_lib_dir: &Path,
) -> PathBuf {
    let mut config = cmake::Config::new(manifest_dir.join("ffi"));
    let urdfdom_headers_dir = find_urdfdom_headers_cmake_dir(dependency_prefix, dependency_lib_dir)
        .expect("failed to locate urdfdom_headers CMake config dir");
    let urdfdom_dir = find_urdfdom_cmake_dir(dependency_lib_dir)
        .expect("failed to locate urdfdom CMake config dir");
    let parallel_jobs = native_parallel_jobs();
    config
        .define(
            "PINOCCHIO_SOURCE_DIR",
            manifest_dir.join("third_party/pinocchio"),
        )
        .define("PINOCCHIO_DEP_PREFIX", dependency_prefix)
        .define("PINOCCHIO_DEP_LIBDIR", dependency_lib_dir)
        .define("urdfdom_headers_DIR", urdfdom_headers_dir)
        .define("urdfdom_DIR", urdfdom_dir)
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_TESTING", "OFF")
        .define("BUILD_BENCHMARK", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_UTILS", "OFF")
        .define("BUILD_PYTHON_INTERFACE", "OFF")
        .define("BUILD_WITH_LIBPYTHON", "OFF")
        .define("BUILD_WITH_SDF_SUPPORT", "OFF")
        .define("BUILD_WITH_COLLISION_SUPPORT", "OFF")
        .define("BUILD_WITH_AUTODIFF_SUPPORT", "OFF")
        .define("BUILD_WITH_CODEGEN_SUPPORT", "OFF")
        .define("BUILD_WITH_OPENMP_SUPPORT", "OFF")
        .define("BUILD_WITH_EXTRA_SUPPORT", "OFF")
        .define("INSTALL_DOCUMENTATION", "OFF")
        .define("ENABLE_TEMPLATE_INSTANTIATION", "ON")
        .define("FETCHCONTENT_UPDATES_DISCONNECTED", "ON")
        // Avoid printing one line per header on `cmake --install` (default is VERY noisy).
        .define("CMAKE_INSTALL_MESSAGE", "LAZY")
        .env("CMAKE_BUILD_PARALLEL_LEVEL", parallel_jobs.to_string())
        .build_arg(format!("-j{parallel_jobs}"));

    config.build()
}

fn native_parallel_jobs() -> usize {
    env::var("AIRBOT_PINOCCHIO_BUILD_JOBS")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .filter(|value| *value > 0)
        .or_else(|| {
            env::var("NUM_JOBS")
                .ok()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|value| *value > 0)
        })
        .or_else(|| std::thread::available_parallelism().ok().map(usize::from))
        .unwrap_or(1)
}

fn detect_pinocchio_link_mode(native_lib_dir: &Path) -> PinocchioLinkMode {
    if native_lib_dir.join("libpinocchio_parsers.a").exists()
        && native_lib_dir.join("libpinocchio_default.a").exists()
    {
        PinocchioLinkMode::Static
    } else {
        PinocchioLinkMode::Dynamic
    }
}

impl PinocchioLinkMode {
    fn rustc_link_kind(self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::Dynamic => "dylib",
        }
    }
}

fn find_pinocchio_dependency_lib_dir(prefix: &Path) -> io::Result<PathBuf> {
    let lib_root = prefix.join("lib");
    let mut candidates = Vec::new();

    if lib_root.is_dir() {
        candidates.push(lib_root.clone());
        for entry in fs::read_dir(&lib_root)? {
            let path = entry?.path();
            if path.is_dir() {
                candidates.push(path);
            }
        }
    }

    for candidate in candidates {
        if candidate
            .join("urdfdom/cmake/urdfdom-config.cmake")
            .exists()
            || candidate.join("pkgconfig/urdfdom.pc").exists()
            || candidate.join("liburdfdom_model.so").exists()
            || candidate.join("liburdfdom_model.a").exists()
        {
            return Ok(candidate);
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "unable to locate dependency library dir under {}",
            prefix.display()
        ),
    ))
}

fn find_urdfdom_headers_include_dir(prefix: &Path) -> io::Result<PathBuf> {
    let candidates = [
        prefix.join("include/urdfdom_headers"),
        prefix.join("include"),
    ];

    for candidate in candidates {
        if candidate.join("urdf_model/model.h").exists() {
            return Ok(candidate);
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "unable to locate urdfdom headers include dir under {}",
            prefix.display()
        ),
    ))
}

fn find_urdfdom_parser_include_dir(prefix: &Path) -> io::Result<PathBuf> {
    let candidates = [prefix.join("include/urdfdom"), prefix.join("include")];

    for candidate in candidates {
        if candidate.join("urdf_parser/urdf_parser.h").exists() {
            return Ok(candidate);
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "unable to locate urdfdom parser include dir under {}",
            prefix.display()
        ),
    ))
}

fn find_urdfdom_headers_cmake_dir(prefix: &Path, lib_dir: &Path) -> io::Result<PathBuf> {
    let candidates = [
        prefix.join("share/urdfdom_headers/cmake"),
        lib_dir.join("urdfdom_headers/cmake"),
        lib_dir.join("urdfdom_headers"),
    ];

    for candidate in candidates {
        if candidate.join("urdfdom_headers-config.cmake").exists()
            || candidate
                .join("urdfdom_headers-config-version.cmake")
                .exists()
        {
            return Ok(candidate);
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "unable to locate urdfdom_headers CMake config dir for {}",
            prefix.display()
        ),
    ))
}

fn find_urdfdom_cmake_dir(lib_dir: &Path) -> io::Result<PathBuf> {
    let candidates = [lib_dir.join("urdfdom/cmake"), lib_dir.join("urdfdom")];

    for candidate in candidates {
        if candidate.join("urdfdom-config.cmake").exists()
            || candidate.join("urdfdom-configVersion.cmake").exists()
        {
            return Ok(candidate);
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "unable to locate urdfdom CMake config dir under {}",
            lib_dir.display()
        ),
    ))
}

fn find_pinocchio_dependency_prefix(manifest_dir: &Path) -> io::Result<PathBuf> {
    if let Ok(prefix) = env::var("AIRBOT_PINOCCHIO_DEP_PREFIX") {
        let path = PathBuf::from(prefix);
        if path.exists() {
            return Ok(path);
        }
    }

    let local_env = manifest_dir.join(".pin-env");
    if local_env.exists() {
        for lib_dir in fs::read_dir(local_env.join("lib"))? {
            let lib_dir = lib_dir?.path();
            if !lib_dir.is_dir()
                || !lib_dir
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .starts_with("python")
            {
                continue;
            }

            let prefix = lib_dir.join("site-packages/cmeel.prefix");
            if prefix.exists() {
                return Ok(prefix);
            }
        }
    }

    let system_prefix = Path::new("/usr");
    if is_pinocchio_dependency_prefix(system_prefix)? {
        return Ok(system_prefix.to_path_buf());
    }

    if let Some(prefix) = find_system_pinocchio_dependency_prefix()? {
        return Ok(prefix);
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        "unable to locate Pinocchio dependency prefix; set AIRBOT_PINOCCHIO_DEP_PREFIX, create .pin-env with cmeel dependencies, or install ROS urdfdom under /opt/ros",
    ))
}

fn find_system_pinocchio_dependency_prefix() -> io::Result<Option<PathBuf>> {
    for name in ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"] {
        if let Some(prefix) = find_prefix_in_env(name)? {
            return Ok(Some(prefix));
        }
    }

    let ros_root = Path::new("/opt/ros");
    if ros_root.is_dir() {
        for entry in fs::read_dir(ros_root)? {
            let prefix = entry?.path();
            if is_pinocchio_dependency_prefix(&prefix)? {
                return Ok(Some(prefix));
            }
        }
    }

    Ok(None)
}

fn find_prefix_in_env(var: &str) -> io::Result<Option<PathBuf>> {
    let Some(paths) = env::var_os(var) else {
        return Ok(None);
    };

    for prefix in env::split_paths(&paths) {
        if is_pinocchio_dependency_prefix(&prefix)? {
            return Ok(Some(prefix));
        }
    }

    Ok(None)
}

fn is_pinocchio_dependency_prefix(prefix: &Path) -> io::Result<bool> {
    Ok(find_pinocchio_dependency_lib_dir(prefix).is_ok()
        && find_urdfdom_headers_include_dir(prefix).is_ok()
        && find_urdfdom_parser_include_dir(prefix).is_ok())
}
