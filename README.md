# airbot-play-rust

Rust library and CLI tools for working with the AIRBOT Play arm over CAN, with
a vendored C++ Pinocchio integration for model-based kinematics and dynamics.

## Overview

This crate contains:

- a reusable Rust library under `src/`
- CLI tools under `bin/`
- Linux host setup payload under `root/` for `udev`, `systemd`, and CAN helper
  scripts
- a source-install helper at `scripts/install-system-setup.sh`
- Debian packaging metadata via `cargo-deb`

The main binaries are:

- `airbot-play-ws`: WebSocket adapter for exposing the robot over a network
  interface
- `airbot-play-e2-ws`: WebSocket adapter that validates the mounted end
  effector is `E2B`
- `airbot-play-g2-ws`: WebSocket adapter that validates the mounted end
  effector is `G2`
- `airbot-play-iceoryx2`: iceoryx2 adapter for shared-memory IPC
- `airbot-play-probe`: probe connected AIRBOT instances and report board
  metadata
- `airbot-can-diag`: decode CAN traffic live or from recorded logs

The library surface includes:

- CAN transport and worker backends
- request/session handling
- probe and diagnostic helpers
- AIRBOT model, arm, motor, and end-effector modules
- warning/event plumbing
- WebSocket transport support
- optional iceoryx2 transport support

## Platform And Dependencies

Linux is the primary supported environment.

This repo builds Pinocchio from the vendored source tree in
`third_party/pinocchio`.

The parent repo carries a local patch at
`patches/pinocchio-static-build.patch` that adds static-library support to the
vendored Pinocchio checkout. Local builds and CI should apply that patch before
compiling.

The build expects native dependencies from the system install, typically under
`/usr`, including:

- `Eigen`
- `Boost` filesystem and serialization
- `urdfdom`
- `console_bridge`
- `tinyxml2`

Default builds also enable the `iceoryx2-transport` feature. That path needs
`clang`, `libclang-dev`, and `llvm-dev` for bindgen.

## Build Prerequisites

On Ubuntu 24.04 / Debian-like systems, the following packages are sufficient
for building the crate:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  clang \
  cmake \
  libclang-dev \
  llvm-dev \
  pkg-config \
  patch \
  libeigen3-dev \
  libboost-filesystem-dev \
  libboost-serialization-dev \
  libboost-test-dev \
  libconsole-bridge-dev \
  libtinyxml2-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev
```

If you also want to install host setup files from the source checkout or use
the packaged helper scripts, install these runtime utilities as well:

```bash
sudo apt-get install -y \
  can-utils \
  iproute2 \
  kmod \
  udev \
  usbutils
```

If a build fails with errors like `fatal error: 'stddef.h' file not found`,
install `clang`, `libclang-dev`, and `llvm-dev`.

## Clone And Build

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
cargo build --locked --all-targets
```

Build only the WebSocket adapter:

```bash
cargo build --locked --bin airbot-play-ws
```

Build the E2 or G2 specific WebSocket adapters:

```bash
cargo build --locked --bin airbot-play-e2-ws
cargo build --locked --bin airbot-play-g2-ws
```

Build only the iceoryx2 adapter:

```bash
cargo build --locked --bin airbot-play-iceoryx2
```

If you do not need iceoryx2 support, you can disable default features:

```bash
cargo build --locked --no-default-features --bin airbot-play-ws
```

## Host Setup From A Source Checkout

If you are using this repo directly as a Rust crate instead of installing the
generated `.deb`, install the host-side CAN setup files from the checkout:

```bash
sudo ./scripts/install-system-setup.sh
```

By default this installs:

- helper scripts under `/usr/local/bin`
- `udev` rules under `/etc/udev/rules.d`
- the `slcan@.service` unit under `/etc/systemd/system`

The script also reloads `udev` and `systemd`, and attempts to load the CAN
kernel modules unless you ask it not to.

Useful options:

- `--prefix /some/path`: install helper scripts under `/some/path/bin`
- `--skip-modprobe`: do not call `modprobe`
- `--skip-reload`: do not reload `udev` rules or `systemd`

After installation, the helper `bind_airbot_device` is available under the
chosen prefix for creating persistent device-name bindings for supported USB CAN
adapters.

## Debian Package

This repo includes a `cargo-deb` configuration that packages the Rust CLI tools
together with the AIRBOT CAN setup helpers under `root/`.

Install the packaging tool:

```bash
cargo install --locked cargo-deb
```

If your environment does not already provide them, install the Debian packaging
helpers used by CI:

```bash
sudo apt-get install -y dpkg-dev liblzma-dev
```

Build the package from a prepared checkout:

```bash
cargo deb --locked
```

The resulting package is written to `target/debian/*.deb` and installs:

- the Rust CLI tools under `/usr/bin`
- helper scripts under `/usr/bin`
- `udev` rules under `/lib/udev/rules.d`
- the `slcan@.service` unit under `/lib/systemd/system`

## Running The Tools

Start the WebSocket adapter:

```bash
cargo run --bin airbot-play-ws -- --interface can0 --bind 127.0.0.1:9002
```

Start the E2-specific WebSocket adapter:

```bash
cargo run --bin airbot-play-e2-ws -- --interface can0 --bind 127.0.0.1:9002
```

Start the G2-specific WebSocket adapter:

```bash
cargo run --bin airbot-play-g2-ws -- --interface can0 --bind 127.0.0.1:9002
```

`E2` is exposed through the existing `MountedEefType::E2B` model path in the
crate, so the E2 wrapper validates against `E2B`.

The `airbot-play-e2-ws` and `airbot-play-g2-ws` entrypoints are EEF-only
servers. They do not bootstrap or enable the arm motors.

Start the iceoryx2 adapter:

```bash
cargo run --bin airbot-play-iceoryx2 -- --interface can0
```

By default, the iceoryx2 service root is derived from the interface as
`airbot-play/<interface>`.

Probe connected AIRBOT devices:

```bash
cargo run --bin airbot-play-probe -- --timeout-ms 1000
```

Probe and emit the structured JSON report:

```bash
cargo run --bin airbot-play-probe -- --json-report
```

Decode live CAN traffic:

```bash
cargo run --bin airbot-can-diag -- --interface can0
```

Decode a recorded `candump` log:

```bash
cargo run --bin airbot-can-diag -- --input /path/to/candump.log
```

Run the websocket keyboard EEF test UI:

```bash
uv run scripts/airbot_play_eef_ws_keyboard.py --url ws://127.0.0.1:9002
```

Spawn the G2 wrapper and control open/close from the keyboard:

```bash
uv run scripts/airbot_play_eef_ws_keyboard.py --spawn-server --server-kind g2 --interface can0
```

Spawn the E2 wrapper and monitor the current value:

```bash
uv run scripts/airbot_play_eef_ws_keyboard.py --spawn-server --server-kind e2 --interface can0
```

In the keyboard UI, `e` enables the end effector, `x` disables it, and for G2
the `o` / `c` keys move the gripper open and closed using G2 MIT commands.

For G2, the runtime keeps replaying the latest MIT target at a fixed rate after
you send it, so host-side control remains active even when you are not pressing
keys continuously.

For E2B hardware, press `e` after connecting. The server keeps feedback flowing
by continuously issuing zero-value E2 MIT commands while the EEF is enabled.

## Dependency Discovery

The build script resolves native dependencies in this order:

1. `AIRBOT_PINOCCHIO_DEP_PREFIX`, if set
2. a local `.pin-env` cmeel prefix, if present
3. the plain system install under `/usr`
4. other prefixes exposed through `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, or
   `/opt/ros`

If you want to force a custom dependency prefix, set:

```bash
export AIRBOT_PINOCCHIO_DEP_PREFIX=/path/to/prefix
```

## CI

The GitHub Actions workflow does two things:

- `build`: installs the native build dependencies, applies
  `patches/pinocchio-static-build.patch`, and builds all targets
- `package-deb`: builds the release binaries, creates a `.deb` with
  `cargo-deb`, and uploads it as a CI artifact

Use that workflow as the reference for a reproducible clean build environment.
