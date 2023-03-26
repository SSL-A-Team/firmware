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
use static_cell::StaticCell;


static INTERRUPT_EXECUTOR: StaticCell<InterruptExecutor<interrupt::I2C1>> = StaticCell::new();


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("kicker startup!");

    let irq = interrupt::take!(I2C1);
    irq.set_priority(interrupt::Priority::P6);
    let executor = INTERRUPT_EXECUTOR.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    loop {}
}