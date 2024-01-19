use nalgebra::Vector3;

pub mod bmi085;
pub trait Imu {
    fn read_accel(&self) -> [u16; 3];
    fn read_gyro(&self) -> [u16; 3];
}

pub type GyroFrame = Vector3<f32>;
pub type AccelFrame = Vector3<f32>;