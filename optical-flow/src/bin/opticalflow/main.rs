#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{gpio::{Level, Output, Speed, Pull}, spi::{Config, Spi}, exti::ExtiInput};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use optical_flow::paa5100je::{self, Paa5100je};
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".axisram.buffers"]
static mut OPTICAL_FLOW_2_BUFFER_CELL: [u8; paa5100je::SPI_MIN_BUF_LEN] = [0; paa5100je::SPI_MIN_BUF_LEN];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut config = Config::default();
    config.frequency = Hertz(2_000_000);
    let mut spi2 = Spi::new(p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH3, p.DMA1_CH4, config);
    let mut spi2_cs = Output::new(p.PA8, Level::High, Speed::VeryHigh);
    let optical_flow_2_buf: &'static mut [u8; paa5100je::SPI_MIN_BUF_LEN] = unsafe { &mut OPTICAL_FLOW_2_BUFFER_CELL };
    let spi2_exti = ExtiInput::new(p.PA9, p.EXTI9, Pull::None);

    //let mut spi1 = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH1, p.DMA1_CH2, config);

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let mut optical_flow_2 = Paa5100je::new_from_spi(spi2, spi2_cs, spi2_exti, optical_flow_2_buf, paa5100je::Rotation::Deg0);
    optical_flow_2.init().await;

    //loop {
    //    let flow = optical_flow_2.read_motion_polling().await;
    //    info!("Flow: {:?}", flow);
//
    //    Timer::after(Duration::from_millis(300)).await;
    //}
}