#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led_red = Output::new(p.PD0, Level::High, Speed::Low);
    let mut led_grn = Output::new(p.PD1, Level::High, Speed::Low);

    let mut en_3v3 = Output::new(p.PB7, Level::High, Speed::Low);
    let mut en_5v0 = Output::new(p.PB8, Level::High, Speed::Low);


    loop {
        info!("high");
        led_red.set_high();
        led_grn.set_high();
        en_3v3.set_high();
        en_5v0.set_high();
        Timer::after_millis(1000).await;

        info!("low");
        led_red.set_low();
        led_grn.set_low();
        en_3v3.set_low();
        en_5v0.set_low();
        Timer::after_millis(1000).await;
    }
}
