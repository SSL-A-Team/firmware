use ateam_controls::robot_model::{KalmanFilterParams, RobotPhysicalParams};
use core::f32::consts::PI;

pub const KF_PARAMS: KalmanFilterParams = KalmanFilterParams {
    process_noise_std_pos_linear: 0.01,
    process_noise_std_pos_angular: 0.05,
    process_noise_std_vel_linear: 0.02,
    process_noise_std_vel_angular: 0.1,
    measurement_noise_std_vision_pos_linear: 1.0,
    measurement_noise_std_vision_pos_angular: 3.14,
    measurement_noise_std_encoder_vel_angular: 50.0,
    measurement_noise_std_gyro_vel_angular: 0.015,
    max_pos_linear: 64.0,
    max_pos_angular: 3.14,
    max_vel_linear: 3.0,
    max_vel_angular: 3.0 * PI,
};

/// alpha: 30 deg (PI/6) - angle between robot y axis and front wheel axis
/// beta: 45 deg (PI/4) - angle between robot y axis and back wheel axis
/// Wheel angles from y+ are [330, 45, 135, 210] degrees
pub const PHYSICAL_PARAMS: RobotPhysicalParams = RobotPhysicalParams {
    alpha: PI / 6.0,                          // 30 degrees
    beta: PI / 4.0,                           // 45 degrees
    l: 0.0814,                                // wheel distance to robot center (m)
    r: 0.030,                                 // wheel radius (m), wheel dia 49mm
    mass: 2.7,                                // robot body mass (kg)
    iz: 0.008,                                // moment of inertia around z axis (kg*m^2)
    motor_torque_constant: 0.0335,            // N*m/A
    motor_efficiency_factor: 13.0,
    coulomb_friction_coefficient_linear: 2.058,
    coulomb_friction_coefficient_angular: 0.05,
    viscous_friction_coefficient_linear: 5.0,
    viscous_friction_coefficient_angular: 0.0063,
};
