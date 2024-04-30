#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Input, Level, Output, Pull, Speed}, spi::{Config, Spi}, time::Hertz,
};
use embassy_time::{Duration, Timer};

use ateam_lib_stm32::drivers::flash::at25df041b::AT25DF041B;

use ateam_kicker_board_v3::{tasks::get_system_config, *};
use ateam_kicker_board_v3::pins::*;

use panic_probe as _;
// use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let sys_config = get_system_config(tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_config);

    info!("kicker startup!");

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let mut spi = Spi::new(p.SPI1, p.PB3, p.PB5, p.PB4, p.DMA2_CH3, p.DMA2_CH2, spi_config);

    let flash = AT25DF041B::new(spi);

    loop {}
}