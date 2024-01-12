pub trait Imu {
    fn read_accel(&self) -> [u16; 3];
    fn read_gyro(&self) -> [u16; 3];
}