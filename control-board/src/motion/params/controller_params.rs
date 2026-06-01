use ateam_controls::{Vector2f, Vector4f, Vector5f};
use embassy_time::Duration;
use nalgebra::Matrix3x5;

// PID gains per row: [Kp, Ki, Kd, Ki_err_min, Ki_err_max]

//////////////////////////////
//  current feedback gains  //
//////////////////////////////

const LINEAR_POSE_PID_GAINS: Vector5f = Vector5f::new(125.0, 0.5, 10.0, -1.0, 1.0);
const ANGULAR_POSE_PID_GAINS: Vector5f = Vector5f::new(125.0, 0.5, 15.0, -1.0, 1.0);

const LINEAR_TWIST_PID_GAINS: Vector5f = Vector5f::new(100.0, 0.0, 0.0, 0.0, 0.0);
const ANGULAR_TWIST_PID_GAINS: Vector5f = Vector5f::new(20.0, 0.0, 0.0, 0.0, 0.0);

pub fn pose_pid_gains() -> Matrix3x5<f32> {
    Matrix3x5::from_rows(&[
        LINEAR_POSE_PID_GAINS.transpose(),
        LINEAR_POSE_PID_GAINS.transpose(),
        ANGULAR_POSE_PID_GAINS.transpose(),
    ])
}

pub fn twist_pid_gains() -> Matrix3x5<f32> {
    Matrix3x5::from_rows(&[
        LINEAR_TWIST_PID_GAINS.transpose(),
        LINEAR_TWIST_PID_GAINS.transpose(),
        ANGULAR_TWIST_PID_GAINS.transpose(),
    ])
}

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

pub const POSE_ACCEL_MODE: PoseAccelMode = PoseAccelMode::Full;

/// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
/// Thresholds for when to recompute the trajectory
pub const TRAJ_RECOMPUTE_ERROR: Vector4f = Vector4f::new(0.5, 1.0, 1.0, 2.0);

/// Accel magnitude threshold for coulomb friction compensation gating.
/// When body_accel_out magnitude is above this, coulomb comp uses target twist direction
/// (helps overcome static friction). Below this, uses deadzoned estimated twist (stable at rest).
/// TODO: separate this to linear and angular
pub const COULOMB_COMP_ACCEL_DEADZONE: f32 = 2.0;

pub const LINEAR_STATE_TWIST_DEADZONE: f32 = 0.05;
pub const ANGULAR_STATE_TWIST_DEADZONE: f32 = 0.3;

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

// Encoder lag model physical parameters.
// K[i]:       DC gain per axis (dimensionless).
// T_SLOPE[i]: dT_eff/d|u| per axis [s / (m/s)].
// N_STEPS:    fixed-point inversion iterations.
pub const ENC_LAG_K: [f32; 2] = [0.95, 0.95];
pub const ENC_LAG_T_SLOPE: [f32; 2] = [0.095, 0.095];
pub const ENC_LAG_T_HORIZON: Duration = Duration::from_millis(10);
