#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(async_closure)]

use defmt_rtt as _;
use defmt::*;
use embassy_stm32::{
    self,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    gpio::{Level, OutputOpenDrain, Pull, Speed, Output},
    usart::{Uart, Parity}, time::mhz,
};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;
use panic_probe as _;
use static_cell::StaticCell;


#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    // setup system clocks
    let mut stm32_config: embassy_stm32::Config = Default::default();
    //stm32_config.rcc.hse = Some(mhz(8));
    //stm32_config.rcc.sys_ck = Some(mhz(400));
    //stm32_config.rcc.hclk = Some(mhz(200));
    //stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    // setup leds
    let mut grn_led = Output::new(p.PB3, Level::High, Speed::Medium);

    grn_led.set_low();

    /////////////////
    //  main loop  //
    /////////////////

    // 100 Hz loop rate
    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    loop {
        grn_led.toggle();

        main_loop_rate_ticker.next().await;
    }
}