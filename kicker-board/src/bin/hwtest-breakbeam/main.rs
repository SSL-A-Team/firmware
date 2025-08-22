#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;

use ateam_kicker_board::tasks::{get_system_config, ClkSource};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use {defmt_rtt as _, panic_probe as _};

use ateam_kicker_board::drivers::breakbeam::Breakbeam;
use embassy_time::{Duration, Timer};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let stm32_config = get_system_config(ClkSource::InternalOscillator);
    let p = embassy_stm32::init(stm32_config);

    let _charge_pin = Output::new(p.PE4, Level::Low, Speed::Medium);
    let _kick_pin = Output::new(p.PE5, Level::Low, Speed::Medium);
    let _chip_pin = Output::new(p.PE6, Level::Low, Speed::Medium);

    info!("breakbeam startup!");

    let mut status_led_green = Output::new(p.PE0, Level::Low, Speed::Medium);
    let mut ball_detected_led1 = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut ball_detected_led2 = Output::new(p.PE3, Level::Low, Speed::Low);

    // Breakbeam
    // nets on schematic are inverted to silkscreen, sorry :/ -Will
    let mut breakbeam = Breakbeam::new(p.PA1, p.PA0);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_high();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(250)).await;

    breakbeam.enable_tx();
    loop {
        // Enable transmitter, wait 100ms, drive blue status LED if receiving, wait 1 sec
        // Timer::after(Duration::from_millis(100)).await;
        if breakbeam.read() {
            ball_detected_led1.set_high();
            ball_detected_led2.set_high();
        } else {
            ball_detected_led1.set_low();
            ball_detected_led2.set_low();
        }
        // Timer::after(Duration::from_millis(1000)).await;

        // Disable transmitter, wait 100ms, drive blue status LED if receiving, wait 1 sec
        // breakbeam.disable_tx();
        // if breakbeam.read()
        // {
        //     status_led_blue.set_high();
        // }
        // else
        // {
        //     status_led_blue.set_low();
        // }
        // Timer::after(Duration::from_millis(1000)).await;

        Timer::after(Duration::from_millis(10)).await;
    }
}
