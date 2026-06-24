use ateam_controls::{Vector2f, Vector3f, Vector4f, Vector5f};
use embassy_time::Duration;
use nalgebra::Matrix3x5;

// PID gains per row: [Kp, Ki, Kd, Ki_err_min, Ki_err_max]

//////////////////////////////
//  current feedback gains  //
//////////////////////////////

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

/// Acceleration (torque) path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN].
/// Scales traj accel and PID feedback respectively in the wheel torque feedforward.
pub const POSE_ACCEL_GAIN: Vector2f = Vector2f::new(1.0, 1.0);

/// Velocity path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN].
/// Scales traj velocity and PID feedback (× dt) respectively in the wheel velocity setpoint.
/// Only active when POSE_VEL_MODE includes feedforward or feedback.
pub const POSE_VEL_GAIN: Vector2f = Vector2f::new(1.0, 1.0);

/// Pose velocity setpoint mode.
///
/// Controls what forms the wheel velocity setpoint.
/// `Disabled`: integrate accel output from estimated twist (original behavior).
/// `FeedforwardOnly`: trajectory velocity directly as the setpoint base, no PID correction.
/// `FeedbackOnly`: integration base plus vel-gain-scaled PID correction term.
/// `Full`: trajectory velocity base plus vel-gain-scaled PID correction.
#[derive(PartialEq, Eq)]
pub enum PoseVelMode {
    Disabled,
    FeedforwardOnly,
    FeedbackOnly,
    Full,
}

pub const POSE_VEL_MODE: PoseVelMode = PoseVelMode::FeedforwardOnly;

/// Pose acceleration (torque feedforward) mode.
///
/// Controls what contributes to the wheel torque feedforward via the accel path.
/// `Disabled`: zero accel output.
/// `FeedforwardOnly`: trajectory acceleration only, no PID feedback.
/// `FeedbackOnly`: PID feedback acceleration only, no trajectory accel.
/// `Full`: trajectory acceleration plus PID feedback (original behavior).
#[derive(PartialEq, Eq)]
pub enum PoseAccelMode {
    Disabled,
    FeedforwardOnly,
    FeedbackOnly,
    Full,
}

pub const POSE_ACCEL_MODE: PoseAccelMode = PoseAccelMode::FeedbackOnly;

/// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
/// Thresholds for when to recompute the trajectory
pub const TRAJ_RECOMPUTE_ERROR: Vector4f = Vector4f::new(0.5, 1.0, 4.0, 8.0);

/// Only applies friction compensation when the absolute value of the
/// velocity/acceleration is above the corresponding threshold in
/// FRICTION_COMP_GATING, to avoid unstable state at 0 velocity
/// [LINEAR_VEL_THRESHOLD, LINEAR_ACCEL_THRESHOLD, ANGULAR_VEL_THRESHOLD, ANGULAR_ACCEL_THRESHOLD]
pub const FRICTION_COMP_GATING: Vector4f = Vector4f::new(0.1, 0.5, 0.5, 1.0);

/// Maximum magnitude of the body-level acceleration output [linear (m/s²), angular (rad/s²)].
/// The friction-compensated acceleration is clamped to these limits before conversion
/// to wheel torques. Prevents runaway current commands from large PID errors.
pub const BODY_ACCEL_CLAMP_LINEAR: f32 = 20.0; // m/s²
pub const BODY_ACCEL_CLAMP_ANGULAR: f32 = 100.0; // rad/s²

/// Maximum magnitude of the body-level velocity output [linear (m/s), angular (rad/s)].
/// Clamped before conversion to wheel velocity setpoints.
pub const BODY_VEL_CLAMP_LINEAR: f32 = 10.0; // m/s
pub const BODY_VEL_CLAMP_ANGULAR: f32 = 50.0; // rad/s

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
//   v_max      = 3.0 m/s  (DEFAULT_KF_MAX_VEL_LINEAR)
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

const VISION_GATE_V_MAX_M_PER_S: f32 = 3.0;
const VISION_GATE_N_MISS: u32 = 3;
const VISION_GATE_T_VISION_S: f32 = 1.0 / 60.0;
const VISION_GATE_T_LATENCY_S: f32 = 0.150;
const VISION_GATE_SLACK_M: f32 = 0.005;

pub const VISION_SEED_SAMPLES: u32 = 20;
pub const VISION_SEED_POS_STD_THRESH_M: f32 = 0.03;
pub const VISION_GATE_BASE_RADIUS_M: f32 = VISION_GATE_V_MAX_M_PER_S
    * (VISION_GATE_N_MISS as f32 * VISION_GATE_T_VISION_S + VISION_GATE_T_LATENCY_S)
    + VISION_GATE_SLACK_M;
pub const VISION_GATE_EXPAND_RATE_M_PER_S: f32 = 2.0;

// Encoder lag model physical parameters.
// K[i]:       DC gain per axis (dimensionless).
// T_SLOPE[i]: dT_eff/d|u| per axis [s / (m/s)].
// N_STEPS:    fixed-point inversion iterations.
pub const ENC_LAG_K: [f32; 2] = [0.95, 0.95];
pub const ENC_LAG_T_SLOPE: [f32; 2] = [0.095, 0.095];
pub const ENC_LAG_T_HORIZON: Duration = Duration::from_millis(10);
