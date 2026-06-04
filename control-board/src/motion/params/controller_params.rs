use ateam_controls::{Vector2f, Vector3f, Vector4f, Vector5f};
use nalgebra::Matrix3x5;

// PID gains per row: [Kp, Ki, Kd, Ki_err_min, Ki_err_max]

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

/// Per-axis anti-jitter thresholds for the body pose PID controller, in the
/// same units as the pose error: [x (m), y (m), theta (rad)]. When the
/// absolute pose error on an axis is below the threshold, the PID output for
/// that axis is linearly scaled toward zero, matching the fixed-point PI
/// anti-jitter behavior on the motor controllers.
pub const POSE_PID_ANTI_JITTER_THRESH: Vector3f = Vector3f::new(0.01, 0.01, 0.02);

/// [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
pub const POSE_CONTROL_GAIN: Vector2f = Vector2f::new(1.0, 1.0);

/// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
/// Thresholds for when to recompute the trajectory
pub const TRAJ_RECOMPUTE_ERROR: Vector4f = Vector4f::new(0.5, 1.0, 10.0, 20.0);

/// [LINEAR_VEL_THRESHOLD, LINEAR_ACCEL_THRESHOLD, ANGULAR_VEL_THRESHOLD, ANGULAR_ACCEL_THRESHOLD]
pub const FRICTION_COMP_GATING: Vector4f = Vector4f::new(0.1, 0.5, 0.5, 1.0);
