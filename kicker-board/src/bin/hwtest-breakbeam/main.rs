#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;

use ateam_kicker_board::tasks::get_system_config;
use embassy_stm32::gpio::{Level, Output, Speed};
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};
use ateam_kicker_board::drivers::breakbeam::Breakbeam;


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let sys_cfg = get_system_config(ateam_kicker_board::tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_cfg);

    let _charge_pin = Output::new(p.PB3, Level::Low, Speed::Medium);
    let _kick_pin = Output::new(p.PB0, Level::Low, Speed::Medium);
    let _chip_pin = Output::new(p.PB1, Level::Low, Speed::Medium);

    info!("breakbeam startup!");
    
    let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
    let mut status_led_blue = Output::new(p.PA8, Level::Low, Speed::Medium);

    // Breakbeam 
    // nets on schematic are inverted to silkscreen, sorry :/ -Will
    let mut breakbeam = Breakbeam::new(p.PA3, p.PA2);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_high();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(250)).await;

    breakbeam.enable_tx();
    loop 
    {
        // Enable transmitter, wait 100ms, drive blue status LED if receiving, wait 1 sec
        // Timer::after(Duration::from_millis(100)).await;
        if breakbeam.read()
        {
            status_led_blue.set_high();
        } 
        else
        {
            status_led_blue.set_low();
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