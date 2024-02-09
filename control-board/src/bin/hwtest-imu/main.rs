#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

mod pins;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_stm32::{
    dma::NoDma,
    spi,
    time::{hz, mhz},
};
use embassy_time::{Duration, Timer};

use apa102_spi::Apa102;
use smart_leds::{SmartLedsWrite, RGB8};

use ateam_control_board::*;


#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));

    info!("system core initialized");

    let p = embassy_stm32::init(stm32_config);

    // Delay so dotstar can turn on
    Timer::after(Duration::from_millis(50)).await;

    let dot_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        NoDma,
        NoDma,
        hz(1_000_000),
        spi::Config::default(),
    );

    let mut dotstar = Apa102::new(dot_spi);
    let _ = dotstar.write([RGB8 { r: 10, g: 0, b: 0 }, RGB8 { r: 0, g: 0, b: 0 }].iter().cloned());

    tasks::imu::start_imu_task(_spawner, p.SPI6, p.PA5, p.PA7, p.PA6, p.BDMA_CH0, p.BDMA_CH1, p.PC4, p.PC5, p.PB1, p.PB2, p.EXTI1, p.EXTI2).expect("unable to start IMU task");

    let _ = dotstar.write([RGB8 { r: 0, g: 0, b: 10 }, RGB8 { r: 0, g: 0, b: 0 }].iter().cloned());


    let mut accel_sub = tasks::imu::get_accel_sub().expect("accel data channel had no subscribers left");
    let mut gyro_sub = tasks::imu::get_gyro_sub().expect("gyro data channel had no subscribers left");

    loop {
        let gyro_data = gyro_sub.next_message_pure().await;
        let accel_data = accel_sub.try_next_message_pure();
        if let Some(accel_data) = accel_data {
            defmt::info!("received gyro ({}, {}, {}) and accel ({}, {}, {})", gyro_data.x, gyro_data.y, gyro_data.z, accel_data.x, accel_data.y, accel_data.z);
        } else {
            defmt::info!("received gyro ({}, {}, {})", gyro_data.x, gyro_data.y, gyro_data.z);
        }

        const MAX_ANGULAR_RATE_RADS: f32 = 34.91;  // IMU task configures for 2000d/s = 34.91 rad/s
        let g_val_mag: u8 = ((gyro_data.z / MAX_ANGULAR_RATE_RADS) * u8::MAX as f32) as u8;

        let _ = dotstar.write([
            RGB8 { r: 0, g: if gyro_data.z > 0.0 { g_val_mag } else { 0 }, b: 0 },
            RGB8 { r: 0, g: if gyro_data.z < 0.0 { g_val_mag } else { 0 }, b: 0 }]
            .iter().cloned());
    }
}
