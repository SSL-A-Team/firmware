#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;

use ateam_kicker_board::tasks::get_system_config;
use embassy_stm32::{adc::{Adc, SampleTime}, gpio::{Level, Output, Speed}};
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;

use embassy_time::{Duration, Timer};
use ateam_kicker_board::drivers::breakbeam::Breakbeam;


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let sys_cfg = get_system_config(ateam_kicker_board::tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_cfg);

    let _charge_pin = Output::new(p.PE4, Level::Low, Speed::Medium);
    let _kick_pin = Output::new(p.PE5, Level::Low, Speed::Medium);
    let _chip_pin = Output::new(p.PE6, Level::Low, Speed::Medium);

    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES112);

    info!("breakbeam startup!");
    
    let mut status_led_green = Output::new(p.PE0, Level::Low, Speed::Medium);
    let mut status_led_blue_left = Output::new(p.PE2, Level::Low, Speed::Medium);
    let mut status_led_blue_right = Output::new(p.PE3, Level::Low, Speed::Medium);


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
    loop 
    {
        // Enable transmitter, wait 100ms, drive blue status LED if receiving, wait 1 sec
        // Timer::after(Duration::from_millis(100)).await;
        if breakbeam.read()
        {
            status_led_blue_left.set_high();
            status_led_blue_right.set_high();

            defmt::info!("ball detected");
        } 
        else
        {
            status_led_blue_left.set_low();
            status_led_blue_right.set_low();

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