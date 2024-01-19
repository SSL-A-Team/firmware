use embassy_sync::{pubsub::Publisher, blocking_mutex::raw::RawMutex};

use crate::drivers::imu::{GyroFrame, AccelFrame};

pub async fn imu_task<'a,
        GyroMutex: RawMutex,
        const GYRO_CAP: usize,
        const GYRO_NSUB: usize,
        const GYRO_NPUB: usize,
        AccelMutex: RawMutex,
        const ACCEL_CAP: usize,
        const ACCEL_NSUB: usize,
        const ACCEL_NPUB: usize>(
    gyro_pub: Publisher<'a, GyroMutex, GyroFrame, GYRO_CAP, GYRO_NSUB, GYRO_NPUB>,
    accel_pub: Publisher<'a, AccelMutex, AccelFrame, ACCEL_CAP, ACCEL_NSUB, ACCEL_NPUB>) {
    
    // BMI085 internally shares a SPI bus so we can't actually asynchronously read both the IMU and Accel
    // unless we used a mutex but that's still a lot of overhead. We'll just sync on the Gyro INT since
    // that gyro data is far more critical for control. When the gyro has an update, we'll also grab the 
    // accel update/

    loop {
        // block on gyro int

        // read gyro data

        // read accel data

    }
}