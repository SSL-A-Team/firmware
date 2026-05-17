# CLAUDE.md

## Project Overview

RoboCup SSL (Small Size League) robot firmware for the A-Team. This is a multi-board embedded system controlling autonomous soccer robots. The firmware runs on several STM32 microcontrollers per robot.

## Repository Structure

```
firmware/
├── control-board/      # Main robot controller (STM32H723ZG, Rust, thumbv7em-none-eabihf)
├── kicker-board/       # Kicker controller (STM32G474VE, Rust, thumbv7em-none-eabihf)
├── motor-controller/   # STSPIN motor controller (STM32F031, C/CMake, thumbv6m-none-eabi)
├── power-board/        # Power management (STM32G030C8, Rust, thumbv6m-none-eabi)
├── controls/           # Git submodule: shared motion-control library (ateam-controls)
├── lib-stm32/          # Shared STM32 embedded library (drivers, radio, IMU, LED, audio)
├── lib-crossarch/      # no_std cross-architecture utilities (math, filters, queues)
├── common/             # Host-side Rust utilities
├── software-communication/  # Packet definitions and radio protocol (C headers + Rust bindgen)
├── credentials/        # WiFi credential crate (private/public)
├── scripts/            # Python telemetry tools (torque_data_writer.py, plot_telemetry.py)
└── util/               # Build helpers (embed_img_hash.py, program.sh)
```

## Build System

**Nix is required.** All builds must run inside a nix develop shell which provides arm-none-eabi-gcc, Rust nightly, cmake, openocd, and probe-rs.

### Build Commands

Make targets follow the pattern: `<module>--<binary>[--<action>]`

```bash
# Enter nix shell and build
nix develop --command make control-board--control    # Build main control binary
nix develop --command make all                       # Build everything
nix develop --command make control-board--control--run  # Build and flash

# Individual modules
nix develop --command make motor-controller--wheel
nix develop --command make motor-controller--dribbler
nix develop --command make kicker-board--kicker
nix develop --command make control-board--all

# Default target (make control) = control-board--control--run
nix develop --command make control

# Without private WiFi credentials
nix develop --command make NO_ATEAM_WIFI_CREDENTIALS=true control-board--all

# Tests
nix develop --command make common--test
nix develop --command make software-communication--test
```

### Build Dependencies

The control-board binary depends on motor-controller and kicker-board binaries being built first (they are embedded into the control binary via `include_bytes!`). The Makefile handles this automatically.

## Architecture

### control-board (Main Board)
The primary robot controller. Runs the motion control loop, state estimation, trajectory planning, and coordinates all subsystems.

- **`src/bin/control/`** — Main production binary
- **`src/bin/hwtest-*/`** — Hardware test binaries (ADC, IMU, motors, radio, kicker, etc.)
- **`src/tasks/`** — Embassy async tasks: `control_task`, `radio_task`, `imu_task`, `kicker_task`, `power_task`, `dotstar_task`, `audio_task`, `user_io_task`
- **`src/motion/`** — Motion control stack:
  - `robot_controller.rs` — `BodyController` orchestrating KF estimation + trajectory tracking + PID control
  - `robot_model.rs` — Local wrappers around `ateam_controls::RobotModel`
  - `pid.rs` — Generic PID controller
  - `params/robot_params.rs` — Robot physical and Kalman filter parameter constants (`KF_PARAMS`, `PHYSICAL_PARAMS`)
  - `constant_gain_kalman_filter.rs` — Legacy constant-gain KF (unused, superseded by controls library KF)
- **`src/drivers/`** — Board-specific drivers (kicker, radio, shell indicator)

### controls (Submodule: ateam-controls)
Shared Rust motion-control library with C FFI bindings. Key components:
- `RobotModel` — 6-state Kalman filter (pos + vel), omni-wheel kinematics, friction model
- `BangBangTraj3D` — 3D bang-bang trajectory planner
- `KalmanFilterParams` / `RobotPhysicalParams` — Parameter structs (no defaults; must be provided by caller)
- Type aliases: `Vector3f`, `Matrix3f`, etc. (nalgebra wrappers)

### motor-controller (C firmware)
STM32F031/STSPIN32-based motor driver. Builds with CMake.
- Binaries: `wheel`, `dribbler`, `wheel-torque`, `wheel-test`, `wheel-torque-test`
- Built binaries are embedded into kicker-board and control-board firmware

### kicker-board
STM32G474 kicker solenoid controller.
- Binaries: `kicker`, `hwtest-blinky`, `hwtest-breakbeam`, `hwtest-charge`, `hwtest-coms`, `hwtest-kick`

### power-board
STM32G030 power management.
- Single binary: `power`
- Tasks: power monitoring, LED, IO, audio, comms

## Key Libraries and Dependencies

- **Embassy** — Async embedded runtime (executor, HAL, time, USB, sync)
- **nalgebra** — Linear algebra (no_std, libm backend)
- **defmt** — Efficient embedded logging
- **ateam-common-packets** — Radio packet types generated via bindgen from C headers
- **libm** — Math functions for no_std

## CI/CD

Three GitHub Actions workflows (`.github/workflows/`):
- **CI.yml** — Full build of all firmware modules in nix
- **rustfmt.yml** — `cargo fmt --check` on all Rust crates
- **rustlint.yml** — `cargo clippy` and `cargo build --features strict` on lib-stm32

## Code Conventions

- Embedded Rust with `#![no_std]`, `#![no_main]`
- Embassy async tasks for concurrency
- `defmt` for logging (not `println!` or `log`)
- Release builds use `opt-level = 3` and `lto = 'fat'`
- All Rust crates use nightly toolchain
- Motor angles: front-left, back-left, back-right, front-right ordering (Vector4)
- Global frame state: [x, y, theta, dx/dt, dy/dt, dtheta/dt] (Vector6)
- Physical units: meters, radians, seconds throughout
