use ateam_controls::{Vector2f, Vector3f, Vector4f, Vector5f};
use embassy_time::Duration;
use nalgebra::{Matrix3x5};

//////////////////////////////////
//  trajectory tracking control  //
//////////////////////////////////

// Pose PID gains per row: [Kp, Ki, Kd, Ki_err_min, Ki_err_max]. The PID acts on
// the trajectory-vs-estimate pose error (with twist error as the derivative) for
// every closed-loop trajectory-tracking maneuver.
const LINEAR_POSE_PID_GAINS: Vector5f = Vector5f::new(300.0, 0.0, 7.0, 0.0, 0.0);
const ANGULAR_POSE_PID_GAINS: Vector5f = Vector5f::new(400.0, 0.0, 30.0, 0.0, 0.0);

pub fn pose_pid_gains() -> Matrix3x5<f32> {
    Matrix3x5::from_rows(&[
        LINEAR_POSE_PID_GAINS.transpose(),
        LINEAR_POSE_PID_GAINS.transpose(),
        ANGULAR_POSE_PID_GAINS.transpose(),
    ])
}

/// Per-axis anti-jitter thresholds for the body pose PID controller, in the
/// same units as the pose error: [x (m), y (m), theta (rad)]. When the
/// absolute pose error on an axis is below the threshold, the PID output for
/// that axis is linearly scaled toward zero, matching the fixed-point PI
/// anti-jitter behavior on the motor controllers.
pub const POSE_PID_ANTI_JITTER_THRESH: Vector3f = Vector3f::new(0.001, 0.001, 0.01);

/// Tracking acceleration (wheel torque) path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN].
/// body accel = FF * trajectory accel + FB * pose PID feedback, so [1, 0] is
/// feedforward only, [0, 1] feedback only and [1, 1] both.
/// Radio-tunable as `TRACKING_ACCEL_GAIN`.
///
/// The tracking velocity (wheel velocity) path has no gains: the body velocity
/// output is always the trajectory velocity. Whether the wheels run their
/// velocity loop on it is chosen by the `wheel_vel_control_enabled` radio bit.
pub const TRACKING_ACCEL_GAIN: Vector2f = Vector2f::new(1.0, 1.0);

/// [CUTOFF_LINEAR_HZ, CUTOFF_ANGULAR_HZ] first-order low-pass cutoff on the
/// trajectory feedforward accel in closed-loop tracking. Smooths bang-bang
/// switching chatter (e.g. replanning to a target that moves in steps).
/// <= 0 disables the filter on that axis group.
/// Radio-tunable as `TRACKING_FF_ACCEL_CUTOFF_HZ`.
pub const TRACKING_FF_ACCEL_CUTOFF_HZ: Vector2f = Vector2f::new(20.0, 20.0);

/// Lookahead (s) for the body velocity output in closed-loop tracking: the
/// trajectory velocity at t = lookahead instead of t = 0. One tick (v(dt))
/// matches the tick-averaged feedforward accel; more covers command and wheel
/// velocity loop latency. Clamped to >= 0.
/// Radio-tunable as `TRACKING_VEL_LOOKAHEAD_S`.
pub const TRACKING_VEL_LOOKAHEAD_S: f32 = 0.001;

/// [ERROR_POS_LINEAR (m), ERROR_POS_ANGULAR (rad), ERROR_VEL_LINEAR (m/s), ERROR_VEL_ANGULAR (rad/s)]
/// Trajectory-vs-estimate errors above which closed-loop tracking is considered
/// diverged, triggering divergence recovery (active brake, then reset).
/// Radio-tunable as `TRACKING_DIVERGENCE_THRESHOLD`.
pub const TRACKING_DIVERGENCE_THRESHOLD: Vector4f = Vector4f::new(1.0, 1.57, 5.0, 10.0);

/// [LINEAR_VEL_THRESHOLD (m/s), ANGULAR_VEL_THRESHOLD (rad/s)]
/// Friction compensation is applied only while the commanded velocity is at or
/// above these thresholds: linear on the local XY speed magnitude, angular on |w|.
/// Radio-tunable as `FRICTION_COMP_VEL_THRESHOLD`.
pub const FRICTION_COMP_VEL_THRESHOLD: Vector2f = Vector2f::new(0.01, 0.001);

/// [ERROR_LINEAR (m), ERROR_ANGULAR (rad)]
/// While friction comp is off for an axis group (commanded speed below
/// FRICTION_COMP_VEL_THRESHOLD) and a closed-loop trajectory is held, breakaway
/// friction comp (Stribeck static, or Coulomb) is applied toward the target
/// when the trajectory-vs-estimate pose error exceeds these and the robot isn't
/// moving away from the target. Lets a stalled robot break free without
/// relying on PID windup. <= 0 disables.
/// Radio-tunable as `FRICTION_COMP_HOLD_ERR_THRESHOLD`.
pub const FRICTION_COMP_HOLD_ERR_THRESHOLD: Vector2f = Vector2f::new(0.005, 0.02);

/// [LINEAR (m/s), ANGULAR (rad/s)] estimated speed away from the target above
/// which hold breakaway friction comp is suppressed. A stalled robot rocks
/// against its wheel/roller compliance; gating on the sign of velocity would
/// toggle the comp with that rocking and amplify it, so only motion away faster
/// than this (e.g. overshoot) turns it off.
pub const FRICTION_COMP_HOLD_AWAY_SPEED: Vector2f = Vector2f::new(0.05, 0.2);

/// Hold breakaway friction comp hysteresis: once on (error above
/// FRICTION_COMP_HOLD_ERR_THRESHOLD) it stays on until the error drops below this
/// fraction of the threshold. Switching off at the threshold itself lets the robot
/// spring back past it and re-trigger, a limit cycle (~5 Hz in theta).
pub const FRICTION_COMP_HOLD_RELEASE_FRAC: f32 = 0.25;

/// Maximum magnitude of the body-level acceleration output [linear (m/s²), angular (rad/s²)].
/// The friction-compensated acceleration is clamped to these limits before conversion
/// to wheel torques. Prevents runaway current commands from large PID errors.
pub const BODY_ACCEL_CLAMP_LINEAR: f32 = 20.0; // m/s²
pub const BODY_ACCEL_CLAMP_ANGULAR: f32 = 100.0; // rad/s²

/// Maximum commanded wheel current (A). The friction-compensated body accel is
/// scaled down uniformly (direction preserved) so no wheel exceeds this. Matches
/// the wheel-torque motor firmware's low-speed limit (MAX_CURR_WHEEL_NOT_TURNING,
/// 2.16 A below 1 rev/s); exceeding it there clips wheels unevenly, which yaws
/// the robot.
pub const WHEEL_CURRENT_LIMIT_A: f32 = 2.16; // A

/// Maximum magnitude of the body-level velocity output [linear (m/s), angular (rad/s)].
/// Clamped before conversion to wheel velocity setpoints.
pub const BODY_VEL_CLAMP_LINEAR: f32 = 10.0; // m/s
pub const BODY_VEL_CLAMP_ANGULAR: f32 = 5.0 * core::f32::consts::TAU; // rad/s (5 RPS), approx. 31.5 rad/s

/// IMU gyroscope full-scale ceiling (BMI088, 2000 deg/s range).
/// BODY_VEL_CLAMP_ANGULAR must stay below this — the KF cannot observe angular
/// velocities beyond this limit, so the state estimate would saturate and drift.
const IMU_GYRO_MAX_RAD_S: f32 = 2000.0 * core::f32::consts::PI / 180.0;
const _: () = assert!(
    BODY_VEL_CLAMP_ANGULAR <= IMU_GYRO_MAX_RAD_S,
    "BODY_VEL_CLAMP_ANGULAR exceeds IMU gyroscope maximum (2000 deg/s)"
);

/// Maximum linear speed (m/s) applied during SSL stop state.
/// Set below the 1.5 m/s rule limit to provide a safety buffer.
/// Applied after control policy output so feedback loops cannot overshoot.
pub const STOP_STATE_LINEAR_SPEED_LIMIT: f32 = 1.25; // m/s (SSL rule limit: 1.5 m/s)

/// Active brake controller parameters (halt and emergency-stop states).
/// Uses raw encoder velocity only — no KF state — so it is robust against
/// state estimator divergence.
///
/// Per-wheel braking current (A): I = clamp(-BRAKE_KP * ω, ±BRAKE_MAX_CURRENT_A)
///
/// Kp derivation: BRAKE_MAX_CURRENT_A / ω_expected_max, targeting current
/// saturation at the top of the expected play-speed range. With 1.0 A and
/// ~100 rad/s typical motor-shaft speed at play speed: Kp ≈ 0.01 A/(rad/s).
///
/// BRAKE_ANTI_JITTER_RADS: output is linearly scaled to zero below this
/// threshold (via PidController anti-jitter) to avoid fighting encoder
/// quantization noise near rest.
///
/// BRAKE_MAX_CURRENT_A must stay below MAX_CURRENT_MA / 1000 (5 A).
pub const BRAKE_KP: f32 = 0.01; // A / (rad/s)
pub const BRAKE_ANTI_JITTER_RADS: f32 = 5.0; // rad/s
pub const BRAKE_MAX_CURRENT_A: f32 = 1.0; // A

/// Trajectory divergence recovery: per-wheel speed (rad/s) below which all
/// wheels are considered stopped, and the number of consecutive ticks they must
/// stay below it for active braking to be "finished" (1 kHz, 1 tick = 1 ms). Once stopped, the controller resets and resumes from the fresh state
/// estimate.
pub const TRACKING_DIVERGENCE_RECOVERY_REST_WHEEL_VEL: f32 = 50.0; // rad/s
pub const TRACKING_DIVERGENCE_RECOVERY_REST_TICKS: u32 = 1;

/// Encoder lag compensation operating mode.
///
/// Intended progression: `Disabled` → `FeedforwardOnly` (validate model params)
///                        → `Full` (add KF correction once params are confirmed).
/// `KfCorrectionOnly` is available for diagnostics but replaces sensor data with model
/// predictions without improving command tracking — enable only with validated params.
#[derive(PartialEq, Eq)]
pub enum EncLagMode {
    /// No lag compensation. Default safe state.
    Disabled,
    /// Pre-compensates wheel commands via model inversion. Graceful degradation if
    /// model params are off — tracking suffers but the estimator is unaffected.
    FeedforwardOnly,
    /// Replaces encoder rows in the KF measurement with the model's predicted reading.
    /// Degrades state estimation if model params are wrong. Validate with
    /// `FeedforwardOnly` before enabling.
    KfCorrectionOnly,
    /// Both feedforward command compensation and KF measurement correction active.
    Full,
}

/// Active encoder lag compensation mode. Change this to enable/disable features.
pub const ENC_LAG_MODE: EncLagMode = EncLagMode::Disabled;

// Vision outlier gate parameters.
//
// BASE_RADIUS is derived from the worst-case prediction error between a valid
// vision measurement and the KF's dead-reckoned position:
//
//   base_radius = v_max * (N_miss * T_vision + T_latency) + slack
//
//   v_max      = 3.0 m/s  (DEFAULT_MAX_VEL_LINEAR)
//   N_miss     = 3         consecutive missed vision packets tolerated
//   T_vision   = 1/60 s   SSL-Vision camera rate
//   T_latency  = 0.150 s  measured worst-case camera→KF pipeline latency
//   slack      = 0.005 m  encoder dead-reckoning error over that window
//
// EXPAND_RATE controls how fast the gate grows when no valid update is received,
// allowing a robot that has been physically repositioned to eventually re-enter
// the gate. At 2.0 m/s the gate reaches the full half-field diagonal (~6 m)
// in ~3 s, which is realistic for a person carrying a robot across the field.
//
// SEED_SAMPLES and SEED_POS_STD_THRESH_M control the startup stability check:
// the robot must receive SEED_SAMPLES consecutive vision measurements whose
// positional standard deviation is below the threshold before the KF is seeded,
// implicitly requiring the robot to be near-stationary at boot.

// Encoder lag model physical parameters.
// K[i]:       DC gain per axis (dimensionless).
// T_SLOPE[i]: dT_eff/d|u| per axis [s / (m/s)].
// N_STEPS:    fixed-point inversion iterations.
pub const ENC_LAG_K: [f32; 2] = [0.95, 0.95];
pub const ENC_LAG_T_SLOPE: [f32; 2] = [0.095, 0.095];
pub const ENC_LAG_T_HORIZON: Duration = Duration::from_millis(10);
