use std::env;
use std::ffi::OsStr;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("missing manifest dir"));
    let dependency_prefix =
        find_pinocchio_dependency_prefix(&manifest_dir).expect("failed to locate Pinocchio deps");

    emit_rerun_for_dir(&manifest_dir.join("ffi")).expect("failed to watch ffi dir");
    emit_rerun_for_dir(&manifest_dir.join("third_party/pinocchio/include"))
        .expect("failed to watch pinocchio headers");
    emit_rerun_for_dir(&manifest_dir.join("third_party/pinocchio/cmake"))
        .expect("failed to watch pinocchio cmake modules");
    emit_rerun_for_dir(&manifest_dir.join("third_party/pinocchio/src"))
        .expect("failed to watch pinocchio sources");
    println!("cargo:rerun-if-changed=src/model/pinocchio_ffi.rs");
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("assets/urdf/play.urdf").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir
            .join("assets/urdf/play_e2.urdf")
            .display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("assets/urdf/play_g2.urdf").display()
    );

    let native_install = build_native(&manifest_dir, &dependency_prefix);
    let native_lib_dir = native_install.join("lib");
    let dependency_lib_dir = dependency_prefix.join("lib");

    cxx_build::bridges(["src/model/pinocchio_ffi.rs"])
        .file(manifest_dir.join("ffi/pinocchio_shim.cpp"))
        .include(&manifest_dir)
        .include(native_install.join("include"))
        .include(manifest_dir.join("third_party/pinocchio/include"))
        .include("/usr/include/eigen3")
        .include(dependency_prefix.join("include/urdfdom"))
        .include(dependency_prefix.join("include/urdfdom_headers"))
        .flag_if_supported("-std=c++17")
        .compile("airbot_pinocchio_ffi");

    println!("cargo:rustc-link-search=native={}", native_lib_dir.display());
    println!("cargo:rustc-link-search=native={}", dependency_lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=pinocchio_parsers");
    println!("cargo:rustc-link-lib=dylib=pinocchio_default");

    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux") {
        for scope in ["cargo:rustc-link-arg", "cargo:rustc-link-arg-tests", "cargo:rustc-link-arg-bins"] {
            println!("{scope}=-Wl,-rpath,{}", native_lib_dir.display());
            println!("{scope}=-Wl,-rpath,{}", dependency_lib_dir.display());
        }
    }
}

fn build_native(manifest_dir: &Path, dependency_prefix: &Path) -> PathBuf {
    let mut config = cmake::Config::new(manifest_dir.join("ffi"));
    let urdfdom_headers_dir = dependency_prefix.join("lib/urdfdom_headers/cmake");
    let urdfdom_dir = dependency_prefix.join("lib/urdfdom");
    let parallel_jobs = native_parallel_jobs();
    config
        .define(
            "PINOCCHIO_SOURCE_DIR",
            manifest_dir.join("third_party/pinocchio"),
        )
        .define("PINOCCHIO_DEP_PREFIX", dependency_prefix)
        .define("urdfdom_headers_DIR", urdfdom_headers_dir)
        .define("urdfdom_DIR", urdfdom_dir)
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
            if !lib_dir.is_dir() || !lib_dir.file_name().unwrap_or_default().to_string_lossy().starts_with("python") {
                continue;
            }

            let prefix = lib_dir.join("site-packages/cmeel.prefix");
            if prefix.exists() {
                return Ok(prefix);
            }
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        "unable to locate Pinocchio dependency prefix; set AIRBOT_PINOCCHIO_DEP_PREFIX or create .pin-env with cmeel dependencies",
    ))
}

fn emit_rerun_for_dir(path: &Path) -> io::Result<()> {
    if !path.exists() {
        return Ok(());
    }

    if path.is_file() {
        println!("cargo:rerun-if-changed={}", path.display());
        return Ok(());
    }

    for entry in fs::read_dir(path)? {
        let entry = entry?;
        let child = entry.path();
        if child.file_name() == Some(OsStr::new(".git")) {
            continue;
        }
        emit_rerun_for_dir(&child)?;
    }

    Ok(())
}
