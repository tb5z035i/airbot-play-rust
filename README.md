# airbot-play-rust

Rust library and CLI tools for working with the AIRBOT Play arm over CAN, with a vendored C++ Pinocchio integration for model-based kinematics and dynamics.

## What is in this repo

The crate exposes a reusable Rust library in `src/` and a few CLI tools in `bin/`:

- `airbot-play-ws`: WebSocket adapter for exposing the robot over a network interface.
- `airbot-play-probe`: Probe and report AIRBOT instances and board metadata.
- `airbot-can-diag`: Decode and inspect CAN traffic live or from recorded logs.

The library surface includes:

- CAN transport and worker backends
- probe and diagnostic helpers
- request/session handling
- AIRBOT model, arm, motor, and end-effector modules
- warning/event plumbing
- WebSocket transport support

## Native build model

This repo builds Pinocchio from the vendored source tree in `third_party/pinocchio`.

The parent repo also carries a local patch at `patches/pinocchio-static-build.patch` that adds static-library support to the vendored Pinocchio checkout. Local builds and CI should apply that patch before compiling.

Pinocchio itself is linked statically into `airbot-play-ws`. The remaining non-Pinocchio native dependencies are resolved from the system install, typically under `/usr`.

Today the build expects system packages for:

- `Eigen`
- `Boost` filesystem and serialization
- `urdfdom`
- `console_bridge`
- `tinyxml2`

## Requirements

Linux is the primary supported environment.

Build prerequisites:

- Rust toolchain with Cargo
- C++ compiler
- `cmake`
- `pkg-config`
- `patch`
- git submodules initialized

On Ubuntu 24.04 / Debian-like systems, the following packages are sufficient for the current build:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  clang \
  cmake \
  libclang-dev \
  pkg-config \
  patch \
  llvm-dev \
  libeigen3-dev \
  libboost-filesystem-dev \
  libboost-serialization-dev \
  libboost-test-dev \
  libconsole-bridge-dev \
  libtinyxml2-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev
```

`clang`, `libclang-dev`, and `llvm-dev` are required for the vendored
`iceoryx2` bindgen steps. If a build fails with errors like
`fatal error: 'stddef.h' file not found`, install those packages first.

## Clone and build

Initialize the submodules first:

```bash
git submodule update --init --recursive
```

Apply the vendored Pinocchio patch:

```bash
patch -d third_party/pinocchio -p1 < patches/pinocchio-static-build.patch
```

Build all targets:

```bash
cargo build --all-targets
```

Or build only the WebSocket adapter:

```bash
cargo build --bin airbot-play-ws
```

## Debian package

This repo includes a `cargo-deb` configuration that packages the Rust CLI tools
together with the AIRBOT CAN setup helpers under `root/`.

Build the package locally from a prepared checkout:

```bash
cargo install cargo-deb --locked
cargo deb --locked
```

The resulting package is written to `target/debian/*.deb` and installs:

- the Rust CLI tools under `/usr/bin`
- the udev rules under `/lib/udev/rules.d`
- the `slcan@.service` unit under `/lib/systemd/system`

## Source checkout host setup

If you are using this repo directly as a Rust crate instead of installing the
generated `.deb`, install the host-side CAN setup files from the checkout:

```bash
sudo ./scripts/install-system-setup.sh
```

By default this installs:

- helper scripts under `/usr/local/bin`
- udev rules under `/etc/udev/rules.d`
- the `slcan@.service` unit under `/etc/systemd/system`

You can override the helper-script prefix with `--prefix /some/path`, or skip
module loading and service reloads with `--skip-modprobe` / `--skip-reload`.

## Running the tools

Start the WebSocket adapter:

```bash
cargo run --bin airbot-play-ws -- --interface can0 --bind 127.0.0.1:9002
```

Probe connected AIRBOT devices:

```bash
cargo run --bin airbot-play-probe -- --timeout-ms 1000
```

Decode live CAN traffic:

```bash
cargo run --bin airbot-can-diag -- --interface can0
```

Decode a recorded `candump` log:

```bash
cargo run --bin airbot-can-diag -- --input /path/to/candump.log
```

## Dependency discovery

The build script resolves native dependencies in this order:

1. `AIRBOT_PINOCCHIO_DEP_PREFIX`, if set
2. a local `.pin-env` cmeel prefix, if present
3. the plain system install under `/usr`
4. other prefixes exposed through `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, or `/opt/ros`

If you want to force a custom dependency prefix, set:

```bash
export AIRBOT_PINOCCHIO_DEP_PREFIX=/path/to/prefix
```

## CI

The repository-level GitHub Actions workflow installs the system packages, applies `patches/pinocchio-static-build.patch`, builds the crate from a clean checkout, and uploads a `.deb` artifact built via `cargo-deb`. Use that workflow as the reference for a reproducible build environment.
