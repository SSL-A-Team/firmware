#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m_rt::entry;

use embassy_executor::Executor;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    time::mhz,
};
use embassy_time::{Duration, Timer};

use static_cell::StaticCell;

use ateam_kicker_board::pins::{ChargePin, RedStatusLedPin, GreenStatusLedPin};

#[embassy_executor::task]
async fn blink( 
        reg_charge: ChargePin,
        status_led_red: RedStatusLedPin,
        status_led_green: GreenStatusLedPin) -> ! {

    let mut reg_charge = Output::new(reg_charge, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(status_led_green, Level::Low, Speed::Medium);
    let mut status_led_red = Output::new(status_led_red, Level::Low, Speed::Medium);

    loop {
        reg_charge.set_low();

        status_led_green.set_high();
        status_led_red.set_low();
        Timer::after(Duration::from_millis(500)).await;
        status_led_green.set_low();
        status_led_red.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.sys_ck = Some(mhz(48));
    stm32_config.rcc.hclk = Some(mhz(48));
    stm32_config.rcc.pclk = Some(mhz(48));

    let p = embassy_stm32::init(stm32_config);

    let _charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let _kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let _chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);

    info!("kicker startup!");
    
    // let mut nvic: NVIC = unsafe { mem::transmute(()) };

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(blink(p.PB3, p.PA12, p.PA11)));
    });
}