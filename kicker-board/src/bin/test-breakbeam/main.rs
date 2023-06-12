#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_stm32::{
    executor::InterruptExecutor,
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt},
    time::mhz,
    usart::{self, Uart},
};
use embassy_time::{Duration, Ticker, Timer};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("breakbeam startup!");
    
    let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
    let mut status_led_red = Output::new(p.PA12, Level::Low, Speed::Medium);
    let mut status_led_blue = Output::new(p.PA8, Level::Low, Speed::Medium);

    // Breakbeam 
    let mut breakbeam_emitter = Output::new(p.PA2, Level::High, Speed::Low);
    let breakbeam_receiver = Input::new(p.PA3, Pull::None);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_high();
    Timer::after(Duration::from_millis(250)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(250)).await;

    loop 
    {
        breakbeam_emitter.set_high();

        if breakbeam_receiver.is_high()
        {
            status_led_blue.set_high();
        } 
        else
        {
            status_led_blue.set_low();
        }
        
    }
}