# Firmware Repository — Copilot Context

## Overview

This is the unified firmware repository for the A-Team SSL (Small Size League) robotics team. It contains firmware for all robot subsystems: control board (STM32H743), motor controllers (STM32F031), kicker board, and supporting libraries. The robot runs a multi-mode position/velocity/acceleration motion control system with closed-loop current (torque) control on the motors.

**Primary build command:** `nix develop --command make control-board--control` (requires nix shell for arm-none-eabi toolchain).

---

## Repository Structure

```
firmware/
├── control-board/          # Main robot controller firmware (STM32H743, Embassy async, Rust)
├── motor-controller/       # Wheel/dribbler motor firmware (STM32F031, C, bare-metal)
├── kicker-board/           # Kicker board firmware (Rust, Embassy)
├── controls/               # Git submodule — ateam-controls library (state estimation, kinematics, trajectory planning)
│   ├── ateam-controls/     # Core Rust library (RobotModel, BangBang, KF)
│   ├── ateam-controls-c/   # C FFI wrapper
│   ├── ateam-controls-py/  # Python ctypes bindings
│   ├── analysis/           # Python tooling for params/visualization
│   └── cpp_tests/          # C++/GTest FFI tests
├── software-communication/ # Git submodule — packet definitions (C headers + Rust bindings)
├── lib-stm32/              # STM32 HAL library (radio drivers, UART, sensors)
├── lib-crossarch/          # no_std cross-arch utilities (queue, filter, math, power)
├── common/                 # Shared Rust utilities
├── scripts/                # Python tooling (telemetry decoding, radio testing, plotting)
├── credentials/            # WiFi credentials (private submodule)
├── util/                   # Build utilities (cmake helpers, flashing scripts)
├── Makefile                # Top-level build orchestration
├── flake.nix               # Nix development environment
└── pyproject.toml           # Python dependencies (uv)
```

### Makefile Target Convention
Targets follow `<module>--<binary>--<action>`:
- `make control-board--control--run` — build all firmware, flash, and run
- `make motor-controller--wheel-torque` — build wheel torque motor firmware
- `make control-board--all` — build all control board binaries
- `make all` — build everything
- `make test` — run all tests (software-communication + common)

### Control Board Binaries
| Binary | Purpose |
|---|---|
| `control` | Production robot firmware |
| `hwtest-motor` | Motor hardware testing |
| `hwtest-torque` | Interactive torque control testing |
| `hwtest-radio` | Odin W26x radio testing |
| `hwtest-radio-w36` | Nora W36x radio testing |
| `profile-wheel-curr` | Wheel current step response capture |
| `profile-wheel-vel` | Wheel velocity step response capture |
| `profile-ang-fric` | Angular friction profiling |
| `profile-lin-fric` | Linear friction profiling |

### Motor Controller Binaries
| Binary | Purpose |
|---|---|
| `wheel` | Legacy open-loop wheel motor firmware |
| `wheel-torque` | Production closed-loop current-controlled wheel firmware |
| `wheel-torque-test` | Current control test harness |
| `wheel-test` | Legacy wheel test firmware |
| `dribbler` | Dribbler motor firmware |

---

## Motion Control Architecture

### High-Level Data Flow

```
Software Stack (PC)
    │
    ▼ (WiFi: BasicControl packet via RadioPacket)
Radio Task (Odin W26x or Nora W36x)
    │
    ▼ (PubSub channel)
Control Task (1 kHz loop)
    │
    ├── BodyController.control_update()
    │       │
    │       ├── Kalman Filter (6-state: [x, y, θ, vx, vy, vθ])
    │       │       └── 8 measurements: 3 vision + 4 encoder + 1 gyro
    │       │
    │       ├── Skill Command Dispatch (position/velocity/acceleration modes)
    │       │       └── Bang-bang trajectory planning + PID feedback
    │       │
    │       ├── Coulomb Friction Compensation
    │       │
    │       └── Kinematic Transforms (body → wheel)
    │               ├── wheel_velocities = T_twist2wheel(θ) × body_twist
    │               └── wheel_torques = T_accel2wheel(θ) × body_accel
    │
    ▼ (UART per motor)
4× CurrentControlledMotor (CcmCommand packets)
    │
    ▼
4× STM32F031 Motor Controller
    └── 6-step commutation + current PI controller (fixed-point)
```

### Control Loop Timing
- **Control loop:** 1 kHz (derived from `DEFAULT_CONTROL_DT = 0.001s`)
- **Basic telemetry:** 100 Hz (every 10 ticks)
- **Extended telemetry:** 100 Hz or immediately on vision update
- **Trace logging:** 10 Hz
- **Packet timeout:** 200 ms
- **Loop overrun warning:** 400 µs

---

## Body Controller (`control-board/src/motion/robot_controller.rs`)

### `BodyController` Struct
Core fields:
- `robot_model: RobotModel` — state estimator + kinematic model
- `pose_pid: PidController<3>` — PID for position control (x, y, θ)
- `twist_pid: PidController<3>` — PID for velocity control (vx, vy, vθ)
- `traj: BangBangTraj3D` — current trajectory
- `traj_time: f32` — trajectory elapsed time
- `prev_skill_cmd` — previous command (for trajectory recomputation detection)
- Output vectors: `body_twist_out`, `body_accel_out`, `body_accel_out_fric_comp`, `wheel_vel_out`, `wheel_torque_out`

### `control_update()` Flow
1. **Vision gating:** Updates vision timeout; if vision arrives, calls `kf_set_pose()` to snap position state
2. **KF update:** Builds 8-element measurement vector `[vision_x, vision_y, vision_θ, wheel_vel_FL, wheel_vel_BL, wheel_vel_BR, wheel_vel_FR, gyro_z]`, calls `kf_update()` with measurement masks
3. **State deadzoning:** Applies deadzone to estimated velocity to prevent drift at rest
4. **Skill command dispatch:** Matches on `BasicControl.get_skill_command()` variant
5. **Friction compensation:** Computes Coulomb + viscous friction force, subtracts via inverse inertia
6. **Kinematic transform:** Converts body twist/accel to wheel velocities/torques using θ-dependent transforms
7. **Telemetry:** Populates `BodyControlTelemetry` and `BodyControlExtendedTelemetry`
8. **KF predict:** Advances state estimate forward one timestep

### Control Modes

#### `GlobalPosition` (Pose Control)
- Requires active vision (200 ms timeout, outputs zero if vision inactive)
- Computes bang-bang trajectory to target pose `[x, y, θ]`
- PID feedback using `calculate_with_derivative()`:
  - Error = target_pose − estimated_pose (θ wrapped to [-π, π] via `remainderf`)
  - Derivative = trajectory_twist − estimated_twist (external derivative, not numerical diff)
- Trajectory recomputed when target changes or actual state deviates beyond `TRAJ_RECOMPUTE_ERROR`
- Outputs: trajectory twist (feedforward) + PID correction (feedback), trajectory accel

#### `GlobalVelocity` (Twist Control)
- Computes bang-bang trajectory to target twist `[vx, vy, vθ]`
- PID feedback on twist error between trajectory state and KF estimate
- Trajectory recomputed on target change or tracking error exceeding thresholds
- Outputs: trajectory twist + PID correction, trajectory accel

#### `LocalVelocity`
- Rotates local target twist into global frame via `z_rotation_mat(θ)`
- Then same logic as GlobalVelocity

#### `GlobalAcceleration`
- Direct passthrough — applies target acceleration through dynamics model `A*x + B*u`
- No trajectory planning or PID feedback

#### `LocalAcceleration`
- Rotates local accel into global frame, then same as GlobalAcceleration

### Trajectory Recomputation
Trajectories are recomputed when:
- The target command changes (different from `prev_skill_cmd`)
- Actual state deviates from tracked trajectory state beyond `TRAJ_RECOMPUTE_ERROR = [0.5, 1.0, 1.0, 2.0]` (x, y, vx, vy thresholds)

### Friction Compensation
Two regimes based on commanded acceleration magnitude:
- **Moving (accel > deadzone):** Uses target twist direction to compute friction force (overcomes static friction)
- **At rest (accel ≤ deadzone):** Uses deadzoned estimated twist for stable equilibrium
- Friction force = `robot_model.compute_friction_force(body_twist)`
- Applied: `body_accel_fric_comp = body_accel - I_inv × friction_force`

### Controller Parameters (`control-board/src/motion/params/controller_params.rs`)
- Pose PID gains: `[P, I, D, I_min, I_max]` per axis (x, y, θ)
- Twist PID gains: same structure
- `POSE_CONTROL_GAIN = [1.0, 1.0]` — feedforward/feedback weighting
- `TRAJ_RECOMPUTE_ERROR = [0.5, 1.0, 1.0, 2.0]`
- `COULOMB_COMP_ACCEL_DEADZONE = 2.0`
- State velocity deadzones for drift suppression

---

## PID Controller (`control-board/src/motion/pid.rs`)

### `PidController<const NUM_STATES: usize>`
- `gain: SMatrix<f32, NUM_STATES, 5>` — columns: `[P, I, D, I_min, I_max]`
- `prev_error: SVector<f32, NUM_STATES>`
- `integrated_error: SVector<f32, NUM_STATES>`

### Methods
- `calculate(setpoint, measurement, dt)` — standard PID with clamped integral
- `calculate_with_derivative(setpoint, measurement, derivative, dt)` — PID using externally provided derivative signal instead of numerical differentiation of error. Used by pose control where the derivative is the twist error between trajectory and estimate.
- `reset()` — zeros error accumulators

---

## Controls Library (`controls/ateam-controls/`)

### RobotModel (`controls/ateam-controls/src/robot_model.rs`)

#### State Vector (6 states)
```
x = [x, y, θ, vx, vy, vθ]
```
- Position in global frame (meters, radians)
- Velocity in global frame (m/s, rad/s)

#### Measurement Vector (8 measurements)
```
z = [vision_x, vision_y, vision_θ, wheel_vel_FL, wheel_vel_BL, wheel_vel_BR, wheel_vel_FR, gyro_z]
```
- Wheel ordering: Front-Left, Back-Left, Back-Right, Front-Right

#### Kalman Filter
- **State transition:** Constant-velocity model, `A` integrates velocity into position over `dt`; `B` maps acceleration input to velocity
- **Process noise `Q`:** Diagonal, configured via `KalmanFilterParams` stds
- **Measurement noise `R`:** Diagonal 8×8
- **Measurement matrix `H`:** Dynamic, updated each tick via `update_h_transform(θ, masks)`
  - Vision rows: identity (rows 0-2) or zero if no vision
  - Encoder rows: `transform_twist2wheel(θ)` (rows 3-6) or zero
  - Gyro row: `H[7,5] = 1` or zero
- **`kf_predict(u)`:** `x = Ax + Bu`, wraps θ, propagates `P`
- **`kf_update(z, masks)`:** Standard Kalman update with innovation, gain, state/covariance update, θ wrapping
- **`kf_set_pose(pose)`:** Snaps position states to vision, collapses position covariance

#### `KalmanFilterParams`
```rust
pos_process_noise_std_linear: 0.01     // position process noise
pos_process_noise_std_angular: 0.05
vel_process_noise_std_linear: 0.02     // velocity process noise
vel_process_noise_std_angular: 0.1
vision_measurement_noise_std_linear: 1.0
vision_measurement_noise_std_angular: 3.14
encoder_measurement_noise_std_wheel_angular_vel: 50.0
gyro_measurement_noise_std_angular_vel: 0.015
max_pos_state_linear: 64.0            // initial covariance bounds
max_pos_state_angular: 3.14
max_vel_state_linear: 3.0
max_vel_state_angular: 3π
```

#### `RobotPhysicalParams`
```rust
alpha: π/6           // front wheel angle from longitudinal axis
beta: π/4            // back wheel angle from longitudinal axis
l: 0.0814            // wheel distance from center (m)
r: 0.03              // wheel radius (m)
mass: 2.7            // robot mass (kg)
iz: 0.008            // moment of inertia about z-axis (kg·m²)
motor_torque_constant: 0.0335   // Nm/A
motor_efficiency_factor: 13.0
coulomb_friction_coeff_linear: 2.058
viscous_friction_coeff_linear: 0.05
coulomb_friction_coeff_angular: 5.0
viscous_friction_coeff_angular: 0.0063
```

#### Kinematic Transforms
The wheel geometry matrix `M` maps body twist to wheel velocities:
```
M = [[-cos(α), -cos(β),  cos(β),  cos(α)],
     [ sin(α), -sin(β), -sin(β),  sin(α)],
     [    l,       l,       l,       l   ]]
```
- `transform_twist2wheel(θ) = Mᵀ × R_z(-θ) / r` — body twist → wheel angular velocities
- `transform_wheel2twist(θ) = r × R_z(θ) × M_inv_pseudoᵀ` — wheel velocities → body twist
- `transform_accel2wheel(θ) = r × M_inv × I × R_z(-θ)` — body accel → wheel torques
- `transform_wheel2accel(θ) = R_z(θ) × I_inv × M / r` — wheel torques → body accel

All transforms are θ-dependent (rotate between body and global frames).

#### Friction Model
```
F_linear = -c_visc_lin × v - c_coul_lin × v̂    (v̂ = unit velocity direction)
T_angular = -c_visc_ang × ω - c_coul_ang × sign(ω)
friction_force = [Fx, Fy, Tz]
```

#### Motor Model
```
wheel_currents = wheel_torques / motor_torque_constant / motor_efficiency_factor
```

### Bang-Bang Trajectory Planner (`controls/ateam-controls/src/bangbang_trajectory.rs`)

#### `TrajectoryParams`
```rust
max_vel_linear: 3.0        // m/s
max_vel_angular: 3π         // rad/s
max_accel_linear: 2.0       // m/s²
max_accel_angular: 2π       // rad/s²
```
Can be overridden per-command from the software stack via the BasicControl packet (falls back to defaults when all zeros).

#### `BangBangTraj3D`
Three independent 1D bang-bang trajectories for x, y, and θ.

Each `BangBangTraj1D` has:
- `sdd1, sdd2, sdd3` — acceleration values for each segment
- `t1, t2, t3, t4` — segment boundary times

#### Profile Types
- **Triangular:** Accel → decel (no coast phase, when max velocity not reached)
- **Trapezoidal:** Accel → coast at max velocity → decel

#### `from_target_pose(init_state, target_pose, params)`
- Iteratively searches angle `α` so x and y trajectories finish simultaneously
- X limits: `cos(α) × max_*_linear`, Y limits: `sin(α) × max_*_linear`
- θ trajectory uses shortest angular path via `wrap_angle(target - init)`
- Handles reverse motion and brake-then-turnaround scenarios

#### `from_target_twist(init_twist, target_twist, params)`
- Computes constant acceleration to reach target twist
- Simpler than pose: just accelerate/decelerate to target velocity

#### Key Methods
- `time_shift(dt)` — shifts all segment times (for trajectory continuation)
- `end_time()` — max of x/y/θ segment end times
- `state_at(current_state, current_time, t)` — evaluates pose + twist at time `t`
- `accel_at(t)` — returns piecewise-constant acceleration command at time `t`

### Default Constants (`controls/ateam-controls/src/defaults.rs`)
All default values for `KalmanFilterParams`, `RobotPhysicalParams`, and `TrajectoryParams` are defined here. The `Default` trait implementations in each struct reference these constants.

### Error Handling
```rust
enum ControlsError {
    InvalidInput = -1,
    SingularMatrix = -2,
    NoSolution = -3,
    InvalidTime = -4,
    ExceedsLimits = -5,
}
```
Errors propagate to the control task, which sets `controls_err` on `SharedRobotState`, locks out motors, and sends error telemetry.

### C FFI (`controls/ateam-controls-c/`)
Exposes all `RobotModel` and trajectory functions as C-compatible functions:
- `ateam_controls_default_kf_params()`, `_phys_params()`, `_traj_params()`
- `ateam_controls_robot_model_new()`, `_kf_predict()`, `_kf_update()`, etc.
- `ateam_controls_traj_from_target_pose()`, `_from_target_twist()`, etc.
- Returns `0` on success, `ControlsError` as `i32` on failure

---

## Motor Current Control (`motor-controller/`)

### Architecture
Each of the 4 wheel motors has a dedicated STM32F031 running `wheel-torque` firmware, communicating with the control board via UART using `CcmCommand`/`CcmResponse` packets.

### Communication Protocol
- **`CcmCommand`** (control board → motor): motion type, velocity setpoint, current setpoint, reset/telemetry flags
- **`CcmResponse`** (motor → control board): telemetry (encoder velocity, current estimate, bus voltage, temperature) or parameter data
- **Command types:** `CCM_CMD_MOTION`, `CCM_CMD_PARAMS`
- **Response types:** `CCM_RESP_TELEM`, `CCM_RESP_PARAMS`

### Motion Control Types (`CcmMotionControlType`)
| Mode | Description |
|---|---|
| `CCM_MCT_MOTOR_OFF` | Motor disabled |
| `CCM_MCT_DUTY_OPENLOOP` | Open-loop duty cycle |
| `CCM_MCT_VOLTAGE_OPENLOOP` | Open-loop voltage |
| `CCM_MCT_CURRENT` | Closed-loop current (torque) control |
| `CCM_MCT_VELOCITY` | Closed-loop velocity control |
| `CCM_MCT_VELOCITY_CURRENT` | Velocity + current combined control |

Production mode is `CCM_MCT_VELOCITY_CURRENT` — the control board sends both wheel velocity and current (torque) setpoints.

### Current Control on STM32F031 (`motor-controller/common/6step_current.c`)

#### 6-Step Commutation
- Hall sensor-based commutation using TIM2 input capture + XOR
- CW/CCW lookup tables map hall state → active phase pair
- Hall transition validation detects sensor errors
- TIM1 PWM with complementary outputs drives the 3 half-bridges

#### Current PI Controller
- Runs in the commutation ISR at the PWM frequency
- Entirely fixed-point arithmetic (STM32F031 has no FPU)
- Fixed-point formats used internally: S7.10, S5.13, S12.0 (with explicit bit-width tracking)
- Named config in code: S12F4 format for the PI controller struct
- PI gains: `kP = 338 × 5`, `kI = 145 × 5`
- Integral clamp: `kI_max/min = ±4095`
- Anti-jitter threshold suppresses output oscillation near zero error

#### Current Sensing (`motor-controller/common/current_sensing.c`)
- **ADC modes:** PWM-triggered DMA (`CS_MODE_PWM_DMA`) and software (`CS_MODE_SOFTWARE`)
- **3 ADC channels:** Motor current, bus voltage, STSPIN temperature
- **Analog front-end:**
  - Sense resistor: 0.05 Ω
  - Op-amp gain: 5.5
  - Zero-current offset voltage: 0.25 V
- **Calibration:** Samples ADC at startup, stores zero-current bias
- **Flash persistence:** Calibration stored at `0x08007C00` with magic `0xAABBCCDD`

### `CurrentControlledMotor` (Control Board Side) (`control-board/src/motor.rs`)
- Manages UART communication with one motor controller
- Handles firmware flashing at startup
- `send_motion_command()` — builds and sends `CcmCommand` with velocity + current setpoints
- `process_packets()` — reads `CcmResponse`, extracts telemetry
- `read_rads()` — returns encoder velocity in rad/s
- `read_current_estimate_ma()` — returns sensed current in mA
- Safety: current clamped to `MAX_CURRENT_MA = 1500 mA` in control task

### Torque-to-Current Conversion
In the control task:
```
wheel_torques = T_accel2wheel(θ) × body_accel_with_friction_comp
wheel_currents = robot_model.torques_to_currents(wheel_torques)
current_ma = wheel_currents × 1000.0   // convert A to mA
clamped = clamp(current_ma, -1500, 1500)
```

---

## Control Task (`control-board/src/tasks/control_task.rs`)

### Initialization
1. Waits for hardware init valid
2. Flashes motor firmware (wheel-torque.bin) to all 4 motors
3. Releases motor reset, enables telemetry
4. Enters periodic loop with `Ticker::every(DEFAULT_CONTROL_DT)`

### Main Loop (per tick)
1. **Process motor UART packets** — reads encoder velocities and current estimates
2. **Drain command packets:**
   - `BasicControl` → updates mode, command, vision data
   - `ParameterCommand` → forwarded to `BodyController::apply_command()`
3. **Read sensor data** — wheel velocities, IMU, kicker, power telemetry
4. **Call `robot_controller.control_update()`** — runs KF + control policy + kinematics
5. **Safety checks** — locks out wheels on: shutdown, controls error, emergency stop, timeout, BCM_OFF
6. **Convert wheel torque → current** — via `torques_to_currents()`, clamp to 1500 mA
7. **Send motor commands** — velocity + current setpoints via UART
8. **Publish telemetry** — rate-limited basic/extended telemetry via PubSub

### Safety / Error Handling
- `controls_err` atomic flag on `SharedRobotState`
- If controls library returns error → flag set, motors locked out, error telemetry sent
- `reset_controller` command clears error and re-enables motion
- `BCM_OFF` mode → zero motor commands, motor type set to `CCM_MCT_MOTOR_OFF`
- Loop timing instrumentation warns on >400 µs execution

---

## Packet Protocol (`software-communication/`)

### Radio Packet Structure
```
RadioPacket = RadioHeader + RadioData (union)
```

### Key Packet Types
| Packet | Direction | Content |
|---|---|---|
| `BasicControl` | PC → Robot | `BodyControlMode` + skill command payload + vision pose + trajectory params |
| `ParameterCommand` | PC → Robot | Parameter read/write for live tuning |
| `BasicTelemetry` | Robot → PC | Body mode, wheel velocities, body velocity estimate, battery %, kicker charge |
| `ExtendedTelemetry` | Robot → PC | Full state: IMU, vision, trajectory, KF state, body commands, per-skill telemetry |
| `ErrorTelemetry` | Robot → PC | Error codes and diagnostic data |

### Skill Commands (in `BasicControl`)
```rust
enum SkillCommand {
    Off,
    GlobalPosition { x, y, θ, max_vel, max_accel },
    GlobalVelocity { vx, vy, vθ, max_accel },
    LocalVelocity { vx, vy, vθ, max_accel },
    GlobalAcceleration { ax, ay, aθ },
    LocalAcceleration { ax, ay, aθ },
}
```
Trajectory parameters (max vel/accel) sent per-command, falling back to defaults when all zeros.

### Parameter Protocol
`ParameterCommand` enables live tuning of:
- `KalmanFilterParams` (process/measurement noise, covariance bounds)
- `RobotPhysicalParams` (mass, inertia, wheel geometry, friction coefficients)
- PID gains (pose and twist)
- Controller parameters (trajectory thresholds, friction deadzone)

---

## Shared Robot State (`control-board/src/robot_state.rs`)

### `SharedRobotState` (Atomic Flags)
- `init_valid` / `hardware_config` — initialization status
- `radio_bridge_ok` — whether radio bridge is connected
- `controls_err` — controls library error (locks out motors)
- `tipped` — robot tipped over detection
- `shutdown` — graceful shutdown flag
- `battery_ok/warn/crit/power_off` — battery status
- `ball_detected` — ball sensor state
- `emergency_stop` — emergency stop engaged

### `RobotState` (Snapshot)
Plain-copy struct created by `get_state()` for use by tasks without atomic access.

---

## Supporting Libraries

### lib-crossarch (`lib-crossarch/`)
`no_std` crate for code portable across architectures:
- **Queue:** Fixed-size SPSC ring buffer with async waker support, eviction semantics, DMA-safe cancel
- **Filters:** `IirFilter` (single-pole exponential), `WindowAveragingFilter` (rolling window, optional soft-init)
- **Math:** `lerp`, `linear_map`, `Number` trait alias for generic numeric ops
- **Power:** `LipoModel` battery percentage estimation from voltage curves

### lib-stm32 (`lib-stm32/`)
STM32 HAL and driver library:
- **Radio drivers:**
  - `w26x/` — Odin W26x (EDM framing + AT commands)
  - `w36x/` — Nora W36x (pure AT command mode, UART baud negotiation 115200→921600, multicast UDP)
- **AT protocol parsers:** Handle OK, ERROR, URCs, binary socket data events
- **Other drivers:** ADC, audio/piezo, bootloader/flash, IMU, LED (DotStar), GPIO switches
- Re-exports lib-crossarch modules (filter, math, power, queue)

---

## Python Tooling

| Script | Purpose |
|---|---|
| `scripts/torque_data_writer.py` | Decodes C struct telemetry from USB serial |
| `scripts/packet_decoder.py` | Decodes RadioPacket format (BasicTelemetry, ExtendedTelemetry, ErrorTelemetry) |
| `scripts/plot_telemetry.py` | Plots decoded telemetry data for offline analysis |
| `scripts/coms_reliability.py` | Radio stress test (packet loss, latency, throughput) |
| `controls/analysis/params.py` | Loads/configures control parameters from JSON files |
| `controls/analysis/build.py` | Builds the controls library |

---

## Clock / Hardware Configuration

- **MCU:** STM32H743 (control board), STM32F031 (motor controllers)
- **PLL3Q:** 48 MHz (USB), **PLL3P:** 192 MHz
- **HSI48:** Synced from USB SOF
- **Packet buffer size:** 80 bytes (accommodates larger telemetry structs)
- **Motor UART:** Per-motor UART channels with DMA

---

## Active Development Context

The most recent major merge (`dev/nick/pose-control → main`) rewrote the motion control architecture:

### What Changed
- **Old:** Single-mode velocity controller, 100 Hz, 5x3 CGKF (3 body velocity states, 5 measurements), PID on velocity, open-loop voltage commutation on motors
- **New:** Multi-mode position/velocity/acceleration controller, 1 kHz, 6-state KF (position + velocity, 8 measurements including vision), bang-bang trajectory planning with PID, closed-loop current (torque) control on motors

### Key Design Decisions
1. **State estimation moved to controls library** — not inline in firmware
2. **Vision integration** — KF uses vision pose when available (200 ms timeout gating)
3. **Trajectory planning** — bang-bang profiles shape acceleration, recomputed on target change or tracking error
4. **Friction compensation** — Coulomb + viscous model compensated in body frame before kinematic transform
5. **Current control** — fixed-point PI on STM32F031, safety-clamped at 1500 mA
6. **`calculate_with_derivative()`** — pose control uses trajectory-vs-estimate twist error as derivative signal instead of numerically differentiating position error (avoids noise amplification)
7. **Nora W36x radio** — new driver for u-blox WiFi module replacing Odin W26x on newer boards
8. **lib-crossarch** — non-STM32 `no_std` code factored out for reuse/testing

### What's Being Tuned
- PID gains for pose and twist control
- Kalman filter noise parameters
- Trajectory max velocity/acceleration limits
- Friction compensation coefficients
- Motor current PI gains
- Trajectory recomputation thresholds
