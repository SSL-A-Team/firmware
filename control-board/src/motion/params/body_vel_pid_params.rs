use nalgebra::{matrix, vector, Matrix3x5, Vector3, Vector4};

// Kp, Ki, Kd, Ki_err_min, Ki_err_max
pub static K_pid: Matrix3x5<f32> = 
        matrix![6.0, 1.5, 0.0, -1.0, 1.0;
                6.0, 1.5, 0.0, -1.0, 1.0;
                6.0, 0.5, 0.0, -3.0, 3.0];

// x, y, theta (m/s, m/s, rad/s)
pub static BODY_VEL_LIM: Vector3<f32> = vector![3.0, 3.0, 18.0];  // 8, 8, 34.9 maxes out motors/IMU measurement rate
pub static BODY_ACC_LIM: Vector3<f32> = vector![5.0, 3.0, 36.0];  // TODO calibrate/ignore
pub static WHEEL_ACC_LIM: Vector4<f32> = vector![2000.0, 2000.0, 2000.0, 2000.0];  // TODO calibrate/ignore