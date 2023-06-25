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
    pwm::{Channel, Channel3Pin}, peripherals::PA2
};
use embassy_time::{Duration, Ticker, Timer};
use ateam_kicker_board::{
    drivers::{breakbeam_pwm::BreakbeamPwm}
};
use ateam_kicker_board::pins::{BlueStatusLedPin, GreenStatusLedPin, BreakbeamTxPin, BreakbeamRxPin};
use embassy_stm32::pwm::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::time::khz;


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let Ch3 = PwmPin::new_ch3(p.PA2);
    let mut pwm = SimplePwm::new(p.TIM2, None, None, Some(Ch3), None, khz(10));
    let max = pwm.get_max_duty();
    pwm.enable(Channel::Ch3);

    info!("PWM initialized");
    info!("PWM max duty {}", max);

    loop {
        pwm.set_duty(Channel::Ch3, 0);
        Timer::after(Duration::from_millis(300)).await;
        pwm.set_duty(Channel::Ch3, max / 4);
        Timer::after(Duration::from_millis(300)).await;
        pwm.set_duty(Channel::Ch3, max / 2);
        Timer::after(Duration::from_millis(300)).await;
        pwm.set_duty(Channel::Ch3, max - 1);
        Timer::after(Duration::from_millis(300)).await;
    }
}
//async fn main(_spawner: Spawner) {
//    let p = embassy_stm32::init(Default::default());
//    info!("breakbeam startup!");
//    
//    let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
//    let mut status_led_blue = Output::new(p.PA8, Level::Low, Speed::Medium);
//
//    // Breakbeam 
//    let breakbeam_int = interrupt::take!(SPI1);
//    let mut breakbeam = BreakbeamPwm::<_,_,_,_>::new_ch3(
//        p.PA2,
//        Channel::Ch3, 
//        p.TIM2,
//        10,
//        p.PA3,
//        breakbeam_int);
//
//    status_led_green.set_high();
//    Timer::after(Duration::from_millis(250)).await;
//    status_led_green.set_low();
//    Timer::after(Duration::from_millis(250)).await;
//    status_led_green.set_high();
//    Timer::after(Duration::from_millis(250)).await;
//    status_led_green.set_low();
//    Timer::after(Duration::from_millis(250)).await;
//
//    loop 
//    {
//        // Enable transmitter, wait until it is ready, drive blue status LED if receiving, wait 1 sec
//        breakbeam.enable_tx();
//        if breakbeam.read()
//        {
//            status_led_blue.set_high();
//        } 
//        else
//        {
//            status_led_blue.set_low();
//        }
//        Timer::after(Duration::from_millis(1000)).await;
//
//        // Disable transmitter, wait until it is ready, drive blue status LED if receiving, wait 1 sec
//        //breakbeam.disable_tx();
//        //if breakbeam.read()
//        //{
//        //    status_led_blue.set_high();
//        //} 
//        //else
//        //{
//        //    status_led_blue.set_low();
//        //}
//        //Timer::after(Duration::from_millis(1000)).await;
//        
//    }
//}