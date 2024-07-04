use nalgebra::{matrix, vector, Matrix3x5, Vector3, Vector4};

// Kp, Ki, Kd, Ki_err_min, Ki_err_max
pub static PID_GAIN: Matrix3x5<f32> = 
        matrix![1.0, 0.0, 0.0, -1.0, 1.0;
                1.0, 0.0, 0.0, -1.0, 1.0;
                2.0, 0.5, 0.0, -2.0, 2.0];

// x, y, theta (m/s, m/s, rad/s)
pub static BODY_VEL_LIM: Vector3<f32> = vector![5.0, 3.0, 22.0];  // 8, 8, 34.9 maxes out motors/IMU measurement rate
pub static BODY_ACC_LIM: Vector3<f32> = vector![5.0, 3.0, 36.0];  // TODO calibrate/ignore

// FL, BL, BR, FR (rad/s^2)
// Rough estimate for peak rating 
// alpha = rated torque / interia
// 8.4 Ncm^2 / (270 gcm^2) = 3110 rad/s^2 actual max
pub static WHEEL_ACC_LIM: Vector4<f32> = vector![2000.0, 2000.0, 2000.0, 2000.0]; 