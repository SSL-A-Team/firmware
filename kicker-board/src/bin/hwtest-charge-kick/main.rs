#![no_std]
#![no_main]

use core::mem;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use embassy_stm32::gpio::{Output, Level, Speed};

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    info!("kicker startup!");
    let mut nvic: NVIC = unsafe { mem::transmute(()) };

    let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
    let mut status_led_red = Output::new(p.PA12, Level::Low, Speed::Medium);

    loop {}
}