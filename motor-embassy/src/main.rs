#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_stm32 as _;
use panic_probe as _;
use defmt_rtt as _;

use defmt::*;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");
    loop {}
}