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
├── lib-stm32-test/         # Tests for lib-stm32 (host-side)
├── lib-crossarch/          # no_std cross-arch utilities (queue, filter, math, power)
├── common/                 # Shared Rust utilities
├── power-board/            # Power board firmware (Rust, Embassy)
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
| `hwtest-adc` | ADC hardware testing |
| `hwtest-bringup` | Board bring-up / hardware sanity checks |
| `hwtest-control` | Control loop hardware testing |
| `hwtest-dotstar` | DotStar LED hardware testing |
| `hwtest-drib` | Dribbler hardware testing |
| `hwtest-imu` | IMU hardware testing |
| `hwtest-io` | GPIO / IO hardware testing |
| `hwtest-kicker` | Kicker hardware testing |
| `hwtest-kicker-coms` | Kicker communication testing |
| `hwtest-llmotor` | Low-level motor hardware testing |
| `hwtest-motor` | Motor hardware testing |
| `hwtest-piezo` | Piezo buzzer hardware testing |
| `hwtest-pivot` | Pivot maneuver hardware testing |
| `hwtest-radio` | Odin W26x radio testing |
| `hwtest-radio-w36` | Nora W36x radio testing |
| `hwtest-torque` | Interactive torque control testing |
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
    │       ├── Maneuver Dispatch (position/velocity/acceleration/pivot modes)
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
- **Packet timeout (`TIME_WITHOUT_PACKET_STOP`):** 0.5 s
- **Vision-active window (`VISION_ACTIVE_TIMEOUT_S`):** 0.5 s
- **Loop overrun warning (`LOOP_EXECUTION_TIME_THRESHOLD_US`):** 600 µs

---

## Motion Module Layout (`control-board/src/motion/`)

After PR #149's skills refactor and PR #152's rename, the old monolithic `robot_controller.rs` is
gone. Motion code is split into:

- **`body_controller.rs`** — top-level `BodyController` owned by the control
  task. Orchestrates per-tick KF update → maneuver dispatch → friction comp →
  kinematic transforms → telemetry → KF predict. Also implements
  `ParameterInterface` (delegating to `ControlContext`).
- **`control_context.rs`** — `ControlContext` owns all controller infra
  (RobotModel/KF, shared `pose_pid_controller`, trajectory + integrated
  `trajectory_state`, gains, gating thresholds, `enc_lag`, `state_estimate`,
  vision timeout, `wheels_disabled`). Exposes the four control policies
  (`pose_control_policy`, `twist_control_policy`, `accel_control_policy`,
  `pivot_control_policy`) and the shared `track_trajectory` helper. Also
  defines `ManeuverSetpoints` (per-tick `{body_twist, body_accel}`) and
  `CommandFrame { Global, Local }`.
- **`maneuvers/`** — one file per maneuver (`global_position`, `global_velocity`,
  `local_velocity`, `global_acceleration`, `local_acceleration`, `pivot`) plus
  `mod.rs` defining the `MotionManeuver` trait (`entry` / `update` / `reset`),
  the `ActiveManeuver` enum, and `ManeuverManager`.
  - `ManeuverManager::tick(cmd, ctx)` watches `BasicControl.body_control_mode`;
    on a mode change it `reset()`s the prior maneuver, constructs the new one
    via `ActiveManeuver::from_maneuver_command`, calls `entry_cmd`, then runs
    `update_cmd` (which calls into the appropriate `*_control_policy`).
- **`params/controller_params.rs`** — body-level constants (PID gains,
  `POSE_*_GAIN/MODE`, `TRAJ_RECOMPUTE_ERROR`, `FRICTION_COMP_GATING`,
  `BODY_*_CLAMP_*`, `ENC_LAG_*`).
- **`pid.rs`** — `PidController<NUM_STATES>` (see below).
- **`robot_model.rs`** — thin wrappers around `ateam_controls::RobotModel`.
- **`constant_gain_kalman_filter.rs`** — legacy, unused.

---

## Body Controller (`control-board/src/motion/body_controller.rs`)

### `BodyController` Struct
Top-level container; almost all logic is delegated:
- `control_context: ControlContext` — owns KF/PID/trajectory/gains/enc_lag/state_estimate
- `maneuver_manager: ManeuverManager` — stateful per-maneuver dispatcher
- Output vectors: `body_twist_out`, `body_accel_out`,
  `body_accel_out_fric_comp`, `wheel_vel_out`, `wheel_torque_out`
- `telemetry: BodyControlTelemetry`, `debug_telemetry: BodyControlExtendedTelemetry`

`wheels_disabled()` and `vision_active()` are pass-throughs to
`control_context`. `wheels_disabled` is reset to `false` at the top of every
`control_update()` and re-asserted by policies that need lockout (currently
`pose_control_policy` when vision isn't active).

### `control_update()` Flow
1. Reset `control_context.wheels_disabled = false`.
2. **`control_context.update_state_estimate()`** — handles vision-transition
   logic (snap KF + drop trajectory + reset PID on inactive → active),
   advances `time_since_vision_update_s`, builds the 8-element measurement
   vector (substituting `enc_lag` predictions for encoder rows when
   `ENC_LAG_MODE` includes `KfCorrectionOnly`/`Full`), calls
   `robot_model.kf_update()`, caches `state_estimate`, returns the pre-update
   `state_prediction` for telemetry.
3. **`maneuver_manager.tick(last_command, &mut control_context)`** — dispatches
   to the active `MotionManeuver::update`, which calls one of the
   `*_control_policy` helpers and returns `(ManeuverSetpoints, ManeuverExtendedTelemetry)`.
4. **Friction compensation:** `control_context.compute_friction(body_twist, body_accel)`
   returns a **global-frame** friction force; `body_accel_out_fric_comp = body_accel_out − I_inv · F_fric_global`.
5. **Kinematic transform:** wheel velocities via `transform_twist2wheel(θ)` (with
   `enc_lag.invert()` pre-compensation on XY when `ENC_LAG_MODE` includes
   `FeedforwardOnly`/`Full`); wheel torques via `transform_accel2wheel(θ)` ×
   friction-compensated accel. `enc_lag.step()` is called whenever
   `ENC_LAG_MODE != Disabled`.
6. **Telemetry:** populates `BodyControlExtendedTelemetry` (vision pose,
   trajectory pos/vel, KF prediction + estimate split into pos/vel,
   commanded twist/accel ± friction comp) and attaches the maneuver telemetry.
7. **`robot_model.kf_predict(body_accel_out)`** — advances the state estimate
   one timestep using the *pre-friction-comp* commanded acceleration.

### Maneuvers and Control Modes

All trajectory-tracking modes share the `track_trajectory` helper on
`ControlContext`: pose PID via `calculate_with_derivative`
(trajectory-vs-estimate twist error as the derivative), FF/FB gating via
`POSE_ACCEL_MODE` / `POSE_VEL_MODE`, weighted by `pose_accel_gain` /
`pose_vel_gain`. Modes differ in **trajectory planner** and (for pose) **vision
gating**. After producing the tracker outputs, `track_trajectory` steps the
integrated `trajectory_state` forward to `t = dt` so the *next* tick's
`tracking_error_exceeded` compares the estimate against where the trajectory
expected the robot to be one tick later.

#### `GlobalPosition` maneuver → `ControlContext::pose_control_policy`
- **Requires active vision.** Without a fresh vision sample (within
  `VISION_ACTIVE_TIMEOUT_S = 0.5 s`) the policy asserts
  `wheels_disabled = true` and returns zero setpoints; upstream forces
  wheel velocity and current commands to zero (motors keep their commanded
  type — they hold zero, not free-wheel).
- Trajectory planner: `BangBangTraj3D::from_target_pose` to target pose `[x, y, θ]`.
- Per-command overrides on `max_linear_vel/angular_vel/linear_acc/angular_acc`
  fall back to `TrajectoryParams::default()` per-field when zero.
- Replans every tick; seed switches to KF estimate when any of the 4 tracking
  errors exceed `TRAJ_RECOMPUTE_ERROR` (`tracking_error_exceeded`).

#### `GlobalVelocity` maneuver → `ControlContext::twist_control_policy(_, Global, _)`
- **Does not require vision.** Always tracks the commanded twist using whatever
  pose estimate the KF currently has (encoder + gyro dead-reckoning when vision
  is absent).
- Trajectory planner: `BangBangTraj3D::from_target_twist` to target twist
  `[vx, vy, vθ]`. The trajectory's pose state is still integrated forward and
  used for replan-error checks via the shared tracker.
- Per-command overrides on `max_linear_acc/max_angular_acc` (vel limits are
  always the defaults). Falls back per-field when zero.
- Replans every tick; seed-state rule identical to `GlobalPosition`.

#### `LocalVelocity` maneuver → `ControlContext::twist_control_policy(_, Local, _)`
- Same as `GlobalVelocity` except the target twist is rotated by
  `z_rotation_mat(θ)` (using the KF estimate's θ) into the global frame
  before planning.

#### `GlobalAcceleration` maneuver → `ControlContext::accel_control_policy(_, Global)`
- Direct passthrough — applies target acceleration through dynamics model
  `next_state = A·x + B·u`; returns `(next_twist, target_accel)`.
- No trajectory planning or PID feedback.

#### `LocalAcceleration` maneuver → `ControlContext::accel_control_policy(_, Local)`
- Rotates local accel into global frame via `z_rotation_mat(θ)`, then same as
  `GlobalAcceleration`.

#### `Pivot` maneuver → `ControlContext::pivot_control_policy`
- Orbits the robot around a fixed center point (e.g., the ball) while pointing
  toward a target angle, using `PivotTrajectory` from `ateam-controls`.
- Per-command overrides on `max_angular_vel`, `max_angular_acc`, and
  `orbit_radius`; `heading_lag` adjusts how far behind the tangent direction
  the robot heads. All fall back to `PivotParams::default()` when zero.
- Replans only when the command values change (`force_replan` logic in
  `PivotManeuver`), unlike the bang-bang modes which replan every tick.

### Vision Behavior (cross-mode)

The body controller's interaction with vision is identical for all trajectory-tracking
modes; only the **gating** differs (pose mode requires vision, velocity mode does not).

| Event                        | Position mode                                                                                                | Velocity mode                                                                                                            |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------ |
| **Start without vision**     | `wheels_disabled()=true`; wheel vel & current forced to 0; controller idle, waiting for first vision.        | Runs normally; KF integrates from default `(0, 0, 0)` pose using encoders + gyro.                                        |
| **First vision arrives**     | KF snaps to vision via `kf_set_pose`; fresh trajectory planned; robot starts tracking.                       | KF snaps to vision; in-flight trajectory dropped and `pose_pid_controller` reset → next tick re-seeds the trajectory from the snapped estimate and the PID restarts clean.  |
| **Vision lost > 0.5 s**      | `wheels_disabled()=true` every tick until vision returns; no state is reset on the transition (the unified vision-reacquisition handler will snap + replan when vision comes back). | Trajectory keeps running on encoder+gyro dead-reckoning; no commanded change.                                            |
| **Vision returns**           | Treated as a fresh start (first-vision path): KF snaps, new trajectory planned, tracking resumes.            | Vision transitioning inactive → active **always** snaps the KF via `kf_set_pose`, drops the in-flight trajectory, and resets `pose_pid_controller`. Next tick replans from the snapped state estimate. |

Key invariants:
- PID integrator and trajectory state are never carried across a vision transition
  that snaps the KF — windup from pre-vision state is cleared.
- `wheels_disabled()` is the only mechanism by which the body controller forces
  wheel commands to zero; the control task ORs it with the existing lockout
  conditions (`stop_wheels()`, packet timeout, `BCM_OFF`).

### Trajectory Recomputation
The trajectory is **replanned every control tick** (1 kHz). The only
question per tick is the **seed state**:

- Seed from the current KF `state_estimate` when:
  - No trajectory exists yet (`trajectory == None`), or
  - Any of the four tracking errors between the integrated trajectory
    state and the KF state estimate exceeds the corresponding
    `TRAJ_RECOMPUTE_ERROR = [pos_lin, pos_ang, vel_lin, vel_ang]`
    threshold (`tracking_error_exceeded`).
- Otherwise seed from the integrated `trajectory_state` so consecutive
  bang-bang profiles hand off continuously without commanding a
  velocity step.

There is no command-change detection or latched previous command; the
plan is regenerated unconditionally from the current target each tick.
Any vision inactive → active transition clears `self.trajectory`, which
routes the next tick's seed through the `state_estimate` branch.

### Friction Compensation
- Friction model is defined in the robot's **local frame** with independent
  coefficients for x (forward/backward) and y (strafe).
- `ControlContext::compute_friction(body_twist, body_accel)` rotates the
  estimated body twist and the commanded body accel into the local frame,
  gates per sub-comp, computes the local-frame friction force, then rotates
  back to global. Returns the **global-frame** friction force; the
  `BodyController` applies `body_accel_out − I_inv · F_fric_global` to get
  `body_accel_out_fric_comp`.
- Per-axis gating via `FRICTION_COMP_GATING = [lin_vel_thresh,
  lin_accel_thresh, ang_vel_thresh, ang_accel_thresh]`:
  - **Linear off** when both `||(local_vx, local_vy)|| < lin_vel_thresh`
    AND `||(local_ax, local_ay)|| < lin_accel_thresh` → zero linear friction.
  - **Angular off** when both `|v_theta| < ang_vel_thresh` AND
    `|a_theta| < ang_accel_thresh` → zero angular friction.
  - When on, the friction force always uses the target (commanded) local
    twist as its velocity direction.
- Friction force = `robot_model.compute_friction_force(local_twist)`
- Applied: `body_accel_fric_comp = body_accel - I_inv × R_z(θ) × friction_force_local`

### Controller Parameters (`control-board/src/motion/params/controller_params.rs`)
PID and gains:
- Pose PID gains `[P, I, D, I_min, I_max]`:
  - Linear (x, y rows): `(300.0, 0.0, 7.0, 0.0, 0.0)`
  - Angular (θ row):    `(400.0, 0.0, 30.0, 0.0, 0.0)`
  - Shared by both pose and twist tracking (single `pose_pid_controller`,
    twist error supplied as the derivative input via
    `calculate_with_derivative`).
- `POSE_PID_ANTI_JITTER_THRESH = [0.001 m, 0.001 m, 0.01 rad]` — per-axis
  linear-scaling deadband on the PID output (matches the fixed-point PI
  anti-jitter on the motor controllers).

Output-path mode gating (replaces the old single `POSE_CONTROL_GAIN`):
- `POSE_ACCEL_GAIN = [FF, FB] = [1.0, 1.0]` — scales trajectory accel and
  PID feedback on the **wheel torque** path.
- `POSE_VEL_GAIN   = [FF, FB] = [1.0, 1.0]` — scales trajectory vel and
  (PID feedback × dt) on the **wheel velocity** setpoint path.
- `POSE_ACCEL_MODE: PoseAccelMode` — `Disabled` | `FeedforwardOnly` |
  `FeedbackOnly` | `Full`. Default: `FeedbackOnly` (torque path = PID feedback only).
- `POSE_VEL_MODE: PoseVelMode` — `Disabled` | `FeedforwardOnly` |
  `FeedbackOnly` | `Full`. Default: `FeedforwardOnly` (wheel-vel setpoint
  is the trajectory velocity directly; PID does not bias the velocity setpoint).
- `Disabled` on the vel path means the velocity setpoint is derived by
  integrating the accel output from the estimated twist (legacy behavior).

Replan thresholds:
- `TRAJ_RECOMPUTE_ERROR = [0.5 m, 1.0 rad, 4.0 m/s, 8.0 rad/s]` —
  `[pos_lin, pos_ang, vel_lin, vel_ang]` integrated-trajectory-vs-estimate
  thresholds. Drives the seed-state choice; the trajectory itself is
  replanned every tick regardless.

Friction comp gating:
- `FRICTION_COMP_GATING = [lin_vel=0.1, lin_acc=0.5, ang_vel=0.5, ang_acc=1.0]`.

Output clamping (applied before kinematic transforms):
- `BODY_ACCEL_CLAMP_LINEAR = 20.0 m/s²` — clamps friction-compensated linear accel magnitude.
- `BODY_ACCEL_CLAMP_ANGULAR = 100.0 rad/s²` — clamps friction-compensated angular accel magnitude.
- `BODY_VEL_CLAMP_LINEAR = 10.0 m/s` — clamps wheel velocity setpoint linear magnitude.
- `BODY_VEL_CLAMP_ANGULAR = 50.0 rad/s` — clamps wheel velocity setpoint angular magnitude.

Encoder-lag compensation (PR #137, currently `Disabled` by default):
- `ENC_LAG_MODE: EncLagMode` — `Disabled` (default) | `FeedforwardOnly`
  (pre-compensate wheel velocity command via model inversion) |
  `KfCorrectionOnly` (replace encoder rows in KF measurement with
  lag-model predictions) | `Full` (both).
- `ENC_LAG_K = [0.95, 0.95]` — per-axis DC gain.
- `ENC_LAG_T_SLOPE = [0.095, 0.095]` — `dT_eff/d|u|` per axis (s / (m/s)).
- `ENC_LAG_T_HORIZON = 10 ms` — sets the fixed-point inversion iteration count.
- Backed by `FirstOrderLag<2>` in `lib-stm32::model`.
- Intended bring-up progression: `Disabled` → `FeedforwardOnly`
  (validate physical params) → `Full` (add KF correction).

---

## PID Controller (`control-board/src/motion/pid.rs`)

### `PidController<const NUM_STATES: usize>`
- `gain: SMatrix<f32, NUM_STATES, 5>` — columns: `[P, I, D, I_min, I_max]`
- `prev_error: SVector<f32, NUM_STATES>`
- `integrated_error: SVector<f32, NUM_STATES>`
- Optional per-axis `anti_jitter_thresh: Option<SVector<f32, NUM_STATES>>` —
  when `|error[i]| < thresh[i]` the output for that axis is **linearly
  scaled** toward zero (matches the fixed-point PI anti-jitter on the motor
  controllers). Constructed via `from_gains_matrix_with_anti_jitter`.

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
pos_process_noise_std_angular: 0.02
vel_process_noise_std_linear: 0.03     // velocity process noise
vel_process_noise_std_angular: 0.04
vision_measurement_noise_std_linear: 0.5
vision_measurement_noise_std_angular: 0.75
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
motor_efficiency_factor: 17.36
coulomb_friction_coeff_linear_x: 0.0
coulomb_friction_coeff_linear_y: 0.0
coulomb_friction_coeff_angular: 0.0
viscous_friction_coeff_linear_x: 0.0
viscous_friction_coeff_linear_y: 0.0
viscous_friction_coeff_angular: 0.0
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

Per local body axis (caller rotates global state into local before invoking,
and rotates the returned force back to global):

```
Fx_local = -c_visc_x · vx_local - c_coul_x · sign(vx_local)
Fy_local = -c_visc_y · vy_local - c_coul_y · sign(vy_local)
Tz       = -c_visc_ang · v_theta - c_coul_ang · sign(v_theta)
friction_force = [Fx, Fy, Tz]   // local frame
```

#### Motor Model
```
wheel_currents = wheel_torques / motor_torque_constant / motor_efficiency_factor
```

### Bang-Bang Trajectory Planner (`controls/ateam-controls/src/bangbang_trajectory.rs`)

#### `TrajectoryParams`
```rust
max_vel_linear: 3.0        // m/s
max_vel_angular: 5π         // rad/s
max_accel_linear: 2.0       // m/s²
max_accel_angular: 10π      // rad/s²
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

#### Velocity PI Controller (PR #137)
- Also runs in the commutation ISR, alongside the current PI controller.
- Same fixed-point anti-jitter behavior as the current PI controller
  (`pid.c`/`pid.h`): when the velocity error is below threshold, the PID
  output is linearly scaled toward zero to suppress idle oscillation.
- The body PID anti-jitter on the control board mirrors this implementation.

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
- Safety: current clamped to `MAX_CURRENT_MA = 2500 mA` in control task

### Torque-to-Current Conversion
In the control task:
```
wheel_torques = T_accel2wheel(θ) × body_accel_with_friction_comp
wheel_currents = robot_model.torques_to_currents(wheel_torques)
current_ma = wheel_currents × 1000.0   // convert A to mA
clamped = clamp(current_ma, -2500, 2500)
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
4. **Call `robot_controller.control_update()`** — KF update + maneuver dispatch via `ManeuverManager` + friction comp + kinematic transforms + KF predict
5. **Safety checks** — locks out wheels on: shutdown, controls error, emergency stop, timeout, BCM_OFF
6. **Convert wheel torque → current** — via `torques_to_currents()`, clamp to 2500 mA
7. **Send motor commands** — velocity + current setpoints via UART
8. **Publish telemetry** — rate-limited basic/extended telemetry via PubSub

### Safety / Error Handling
- `controls_err` atomic flag on `SharedRobotState`
- If controls library returns error → flag set, motors locked out, error telemetry sent
- `reset_controller` command clears error and re-enables motion
- `BCM_OFF` mode → zero motor commands, motor type set to `CCM_MCT_MOTOR_OFF`
- Loop timing instrumentation warns on >600 µs execution (sends an
  `ErrorTelemetry` "control loop execution time too high!")

---

## Packet Protocol (`software-communication/`)

### Radio Packet Structure
```
RadioPacket = RadioHeader + RadioData (union)
```

### Key Packet Types
| Packet | Direction | Content |
|---|---|---|
| `BasicControl` | PC → Robot | `BodyControlMode` + maneuver command payload + vision pose + trajectory params |
| `ParameterCommand` | PC → Robot | Parameter read/write for live tuning |
| `BasicTelemetry` | Robot → PC | Body mode, wheel velocities, body velocity estimate, battery %, kicker charge |
| `ExtendedTelemetry` | Robot → PC | Full state: IMU, vision, trajectory, KF state, body commands, per-maneuver telemetry |
| `ErrorTelemetry` | Robot → PC | Error codes and diagnostic data |

### Maneuver Commands (in `BasicControl`)
```rust
enum ManeuverCommand {
    Off,
    GlobalPosition { x, y, θ, max_vel, max_accel },
    GlobalVelocity { vx, vy, vθ, max_accel },
    LocalVelocity { vx, vy, vθ, max_accel },
    GlobalAcceleration { ax, ay, aθ },
    LocalAcceleration { ax, ay, aθ },
    Pivot { center_x, center_y, global_theta, orbit_radius, max_angular_vel, max_angular_acc, heading_lag },
}
```
Trajectory parameters (max vel/accel) sent per-command, falling back to defaults when all zeros.

### Parameter Protocol
`ParameterCommand` enables live tuning of:
- `KalmanFilterParams` (process/measurement noise, covariance bounds)
- `RobotPhysicalParams` (mass, inertia, wheel geometry, friction coefficients)
- PID gains (pose and twist)
- Controller parameters (trajectory thresholds, friction-comp gating)

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
2. **Vision integration** — KF uses vision pose when available (0.5 s timeout gating)
3. **Trajectory planning** — bang-bang profiles shape acceleration, recomputed on target change or tracking error
4. **Friction compensation** — Coulomb + viscous model compensated in body frame before kinematic transform
5. **Current control** — fixed-point PI on STM32F031, safety-clamped at 2500 mA
6. **`calculate_with_derivative()`** — pose control uses trajectory-vs-estimate twist error as derivative signal instead of numerically differentiating position error (avoids noise amplification)
7. **Nora W36x radio** — new driver for u-blox WiFi module replacing Odin W26x on newer boards
8. **lib-crossarch** — non-STM32 `no_std` code factored out for reuse/testing

### Recent Changes (post pose-control merge)
- **PR #137 — wheel velocity PID + anti-jitter:** Added a velocity PI loop
  alongside the current PI on each STM32F031 motor controller, both using a
  fixed-point linear-scaling anti-jitter near zero error. Mirrored on the
  control board via `PidController::from_gains_matrix_with_anti_jitter`
  and `POSE_PID_ANTI_JITTER_THRESH`.
- **PR #137 — encoder-lag model:** New `FirstOrderLag<N>` in
  `lib-stm32/src/model/` with `FeedforwardOnly` / `KfCorrectionOnly` /
  `Full` gating via `ENC_LAG_MODE`. Default is `Disabled`; intended
  progression is `Disabled` → `FF` → `Full`.
- **PR #137 — accel/vel mode gating:** Replaced the single
  `POSE_CONTROL_GAIN` with independent `POSE_ACCEL_GAIN` /
  `POSE_VEL_GAIN` plus `POSE_ACCEL_MODE` / `POSE_VEL_MODE` enums.
  Default config: `POSE_ACCEL_MODE = FeedbackOnly`,
  `POSE_VEL_MODE = FeedforwardOnly`.
- **PR #139 — firmware/software hash handshake:** `build.rs` injects
  `GIT_FIRMWARE_HASH`, `GIT_CONTROLS_HASH`, `GIT_COMS_HASH` (+ dirty
  flags) at compile time; `git_version::{FIRMWARE,CONTROLS,COMS}_HASH`
  are 4-byte truncated SHAs reported to the software stack over radio
  (`radio_robot.rs`, `radio_robot_nora.rs`) so the PC side can detect
  firmware mismatches.
- **PR #144 — controller unification & tuning:** Pose and twist policies
  now share a single `pose_pid_controller` and `track_trajectory`
  function. Updated `TRAJ_RECOMPUTE_ERROR` to `[0.5, 1.0, 4.0, 8.0]` and
  `FRICTION_COMP_GATING` to `[0.1, 0.5, 0.5, 1.0]`. Pose PID gains:
  linear `(300, 0, 7)`, angular `(400, 0, 30)`.
- **PR #145 — replan from trajectory state:** On replan with healthy
  tracking, seed the new bang-bang from `trajectory_state` (continuous
  handoff) instead of the KF estimate; fall back to the KF estimate only
  when `tracking_error_exceeded`.
- **Always-replan policy (PR #145):** The trajectory is now
  replanned every control tick. Command-change detection
  (`TRAJ_REPLAN_CMD_*`, `prev_body_cmd`) was removed — the only thing
  that varies per tick is the seed state (trajectory state vs. state
  estimate, per the rule above).
- **PR #149 — skills refactor + timing bugfixes:** Split the monolithic
  `robot_controller.rs` into `body_controller.rs` (top-level orchestrator),
  `control_context.rs` (`ControlContext` owns KF/PID/trajectory/gains/
  enc_lag/state_estimate and exposes `pose_control_policy` /
  `twist_control_policy` / `accel_control_policy`), and one file per skill
  under `skills/` behind a stateful `MotionSkill` trait
  (`entry` / `update` / `reset`) dispatched by `SkillManager` on
  `body_control_mode` changes. Bugfixes/timing: vision-active window and
  packet-stop timeout both raised from 0.2 s → 0.5 s
  (`VISION_ACTIVE_TIMEOUT_S`, `TIME_WITHOUT_PACKET_STOP`); loop-overrun
  warning threshold raised from 400 µs → 600 µs
  (`LOOP_EXECUTION_TIME_THRESHOLD_US`); unused `trajectory_time` removed
  from the controller state. `track_trajectory` now steps
  `trajectory_state` forward to `t = dt` after producing outputs so the
  next tick's `tracking_error_exceeded` compares against the one-step-ahead
  trajectory state.
- **PR #152 — rename skills → maneuvers:** Renamed `skills/` directory to
  `maneuvers/`, `MotionSkill` trait to `MotionManeuver`, `SkillSetpoints` to
  `ManeuverSetpoints`, `SkillExtendedTelemetry` to `ManeuverExtendedTelemetry`,
  `ActiveSkill` to `ActiveManeuver`, and `SkillManager` to `ManeuverManager`.
  `ManeuverCommand` replaces `SkillCommand` in the packet protocol.
- **PR #155 — tighter anti-jitter thresholds:** Reduced
  `POSE_PID_ANTI_JITTER_THRESH` from `[0.01 m, 0.01 m, 0.02 rad]` to
  `[0.001 m, 0.001 m, 0.01 rad]` (1 mm / 0.5 deg) for finer position
  hold without oscillation.
- **PR #156 — Pivot maneuver:** Added `PivotManeuver` under `maneuvers/pivot.rs`
  backed by `PivotTrajectory` / `PivotParams` in `ateam-controls`. The robot
  orbits a fixed center point while facing a target angle. Replans only on
  command change (not every tick). Adds `BCM_PIVOT` body control mode,
  `PivotCommand` / `PivotTelemetry` packets, and
  `ControlContext::pivot_control_policy`. Also adds `hwtest-pivot` binary.

### What's Being Tuned
- PID gains for pose and twist control
- Kalman filter noise parameters
- Trajectory max velocity/acceleration limits
- Friction compensation coefficients
- Motor current and velocity PI gains
- Trajectory recomputation thresholds
- Encoder-lag model parameters (`ENC_LAG_K`, `ENC_LAG_T_SLOPE`) ahead of
  enabling `ENC_LAG_MODE` beyond `Disabled`
