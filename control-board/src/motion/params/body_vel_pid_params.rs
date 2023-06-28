use nalgebra::{matrix, vector, Matrix3x5, Vector3, Vector4};

// Kp, Ki, Kd, Ki_err_min, Ki_err_max
pub static K_pid: Matrix3x5<f32> = 
        matrix![1.0, 0.0, 0.0, 0.0, 0.0;
                1.0, 0.0, 0.0, 0.0, 0.0;
                1.0, 0.0, 0.0, 0.0, 0.0];

// x, y, theta (m/s, m/s, rad/s)
pub static BODY_VEL_LIM: Vector3<f32> = vector![8.0, 8.0, 34.9];  // 8, 8, 34.9 maxes out motors/IMU measurement rate
pub static BODY_ACC_LIM: Vector3<f32> = vector![1.0, 1.0, 1.0];  // TODO calibrate/ignore
pub static WHEEL_ACC_LIM: Vector4<f32> = vector![1.0, 1.0, 1.0, 1.0];  // TODO calibrate/ignore