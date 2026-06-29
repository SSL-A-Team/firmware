use crate::motion::params::controller_params::{
    BRAKE_ANTI_JITTER_RADS, BRAKE_KP, BRAKE_MAX_CURRENT_A,
};
use crate::motion::pid::PidController;
use ateam_controls::Vector4f;
use nalgebra::SMatrix;

/// Active braking controller for halt and emergency-stop states.
///
/// Drives each wheel to zero velocity using a P controller on raw encoder
/// readings. KF-independent: no state estimator state is used, so this
/// remains effective even under KF divergence.
///
/// Output is per-wheel current in Amperes (counter-direction to current velocity).
/// The PidController provides anti-jitter near rest and hard output clamping.
/// Ki and Kd are zero; add them here if proportional-only stopping is too slow.
pub struct ActiveBrakeController {
    pid: PidController<4>,
}

impl ActiveBrakeController {
    pub fn new() -> Self {
        // Gain rows: [Kp, Ki, Kd, Ki_err_min, Ki_err_max]
        let gains = SMatrix::<f32, 4, 5>::from_row_slice(&[
            BRAKE_KP, 0.0, 0.0, 0.0, 0.0,
            BRAKE_KP, 0.0, 0.0, 0.0, 0.0,
            BRAKE_KP, 0.0, 0.0, 0.0, 0.0,
            BRAKE_KP, 0.0, 0.0, 0.0, 0.0,
        ]);
        let anti_jitter = Some(Vector4f::from_element(BRAKE_ANTI_JITTER_RADS));
        let limits = Some((
            Vector4f::from_element(-BRAKE_MAX_CURRENT_A),
            Vector4f::from_element(BRAKE_MAX_CURRENT_A),
        ));
        let mut pid =
            PidController::from_gains_matrix_with_anti_jitter(&gains, anti_jitter);
        pid.set_output_limits(limits);
        ActiveBrakeController { pid }
    }

    /// Returns per-wheel braking currents in Amperes.
    ///
    /// Setpoint is zero (stop); process variable is raw encoder velocity.
    /// dt_s is the control loop period; only matters if Ki or Kd are non-zero.
    pub fn compute(&mut self, wheel_vel_meas: Vector4f, dt_s: f32) -> Vector4f {
        self.pid.calculate(&Vector4f::zeros(), &wheel_vel_meas, dt_s)
    }

    pub fn reset(&mut self) {
        self.pid.reset();
    }
}
