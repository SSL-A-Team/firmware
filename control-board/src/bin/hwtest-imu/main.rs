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
    gpio::{Level, Output, Speed},
    spi,
    time::{hz, mhz},
};
use embassy_time::{Duration, Timer};

use apa102_spi::Apa102;
use smart_leds::{SmartLedsWrite, RGB8};

use ateam_control_board::tasks::imu::imu_task;

#[link_section = ".sram4"]
static mut SPI6_BUF: [u8; 4] = [0x0; 4];

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
    let _ = dotstar.write([RGB8 { r: 10, g: 0, b: 0 }].iter().cloned());

    let mut imu_spi = spi::Spi::new(
        p.SPI6,
        p.PA5,
        p.PA7,
        p.PA6,
        p.BDMA_CH0,
        p.BDMA_CH1,
        hz(1_000_000),
        spi::Config::default(),
    );

    static GYRO_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 1, 1, 1> = PubSubChannel::new();
    static ACCEL_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 1, 1, 1> = PubSubChannel::new();

    spawn_imu_tasks(_spawner, GYRO_CHANNEL.publisher(), ACCEL_CHANNEL.publisher());
}
