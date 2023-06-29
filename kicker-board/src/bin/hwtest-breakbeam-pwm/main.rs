#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::AtomicU32;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt},
    time::mhz,
    usart::{self, Uart},
    pwm::{Channel, Channel3Pin}
};
use embassy_time::{Duration, Ticker, Timer};
use ateam_kicker_board::{
   drivers::{breakbeam_pwm::BreakbeamPwm}
};
use ateam_kicker_board::pins::{BlueStatusLedPin, GreenStatusLedPin, BreakbeamTxPin, BreakbeamRxPin};
use embassy_stm32::pwm::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::time::khz;
use static_cell::StaticCell;
use embassy_executor::InterruptExecutor;

static EXECUTOR_BREAKBEAM_PWM_RX: StaticCell<InterruptExecutor<interrupt::typelevel::I2C1>> = StaticCell::new();
static BREAKBEAM_RX_COUNT: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn poll_counter() {
    loop {
        let shared_var = BREAKBEAM_RX_COUNT.load(Ordering::Relaxed);
        BREAKBEAM_RX_COUNT.store(shared_var.wrapping_add(1), Ordering::Relaxed);
    }
}


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("breakbeam startup!");
    
    let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
    let mut status_led_blue = Output::new(p.PA8, Level::Low, Speed::Medium);


    // Breakbeam 
    let mut breakbeam = BreakbeamPwm::<_,_,_>::new_ch3(
        p.PB0,
        Channel::Ch3, 
        p.TIM3,
        10,
        p.PA3);

    let breakbeam_irq = interrupt::take!(I2C1);
    breakbeam_irq.set_priority(interrupt::typelevel::Priority::P6);
    let executor = EXECUTOR_BREAKBEAM_PWM_RX.init(InterruptExecutor::new(breakbeam_irq));
    let spawner = executor.start();
    info!("start");

    spawner.spawn(poll_counter()).unwrap();

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
        // Enable transmitter, wait until it is ready, drive blue status LED if receiving, wait 1 sec
        breakbeam.enable_tx();
        let count = BREAKBEAM_RX_COUNT.load(Ordering::Relaxed);
        BREAKBEAM_RX_COUNT.store(0, Ordering::Relaxed);

        info!("count:{}", count);
        if breakbeam.read()
        {
            status_led_blue.set_high();
        } 
        else
        {
            status_led_blue.set_low();
        }
        Timer::after(Duration::from_millis(100)).await;

        // Disable transmitter, wait until it is ready, drive blue status LED if receiving, wait 1 sec
        breakbeam.disable_tx();
        if breakbeam.read()
        {
            status_led_blue.set_high();
        } 
        else
        {
            status_led_blue.set_low();
        }
        Timer::after(Duration::from_millis(100)).await;
        
    }
}