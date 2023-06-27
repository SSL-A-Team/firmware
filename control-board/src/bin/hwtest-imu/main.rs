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

    // // acceleromter
    let mut imu_cs1 = Output::new(p.PC4, Level::High, Speed::VeryHigh);
    // // gyro
    let mut imu_cs2 = Output::new(p.PC5, Level::High, Speed::VeryHigh);

    Timer::after(Duration::from_millis(1)).await;

    unsafe {
        SPI6_BUF[0] = 0x80;
        // info!("xfer {=[u8]:x}", SPI6_BUF[0..1]);
        imu_cs1.set_low();
        let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
        imu_cs1.set_high();
        let accel_id = SPI6_BUF[1];
        info!("accelerometer id: 0x{:x}", accel_id);

        SPI6_BUF[0] = 0x80;
        imu_cs2.set_low();
        let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
        imu_cs2.set_high();
        let gyro_id = SPI6_BUF[1];
        info!("gyro id: 0x{:x}", gyro_id);

        loop {
            SPI6_BUF[0] = 0x86;
            // SPI6_BUF[0] = 0x86;
            imu_cs2.set_low();
            let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..3]).await;
            imu_cs2.set_high();
            let rate_z = (SPI6_BUF[2] as u16 * 256 + SPI6_BUF[1] as u16) as i16;
            info!("z rate: {}", rate_z);
        }
    }
}
