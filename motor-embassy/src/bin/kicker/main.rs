#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(const_mut_refs)]
#![feature(ptr_metadata)]

use defmt::*;
use defmt_rtt as _;

use embassy_stm32::gpio::{Output, Level, Speed};
use embassy_stm32::time::mhz;
use embassy_stm32::{self as _,};
use embassy_time::{Duration, Timer};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    // PF9: MOTOR_AUX_RST
    let mut output = Output::new(p.PF9, Level::Low, Speed::High);
    output.set_high();
    Timer::after(Duration::from_micros(500)).await;
    output.set_low();
}
