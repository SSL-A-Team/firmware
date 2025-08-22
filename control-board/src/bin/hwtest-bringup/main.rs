#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use ateam_control_board::get_system_config;
use ateam_lib_stm32::drivers::led::apa102::Apa102;
use embassy_stm32::gpio::{Level, Output, Speed};

const DOTSTAR_BUF_SIZE: usize = 8 + (11 * 4);

#[link_section = ".sram4"]
static mut DOTSTAR_SPI_BUFFER_CELL: [u8; DOTSTAR_BUF_SIZE] = [0; DOTSTAR_BUF_SIZE];

use embassy_time::Timer;
use smart_leds::colors::{BLUE, WHITE};
// provide embedded panic probe
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_main_spawner: embassy_executor::Spawner) {
    // init system
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("embassy HAL configured.");

    let mut led3 = Output::new(p.PG4, Level::Low, Speed::Low);
    let mut led2 = Output::new(p.PG5, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PG6, Level::Low, Speed::Low);

    // Get the pins from the schematic
    let dotstar_spi_buf: &'static mut [u8; DOTSTAR_BUF_SIZE] =
        unsafe { &mut DOTSTAR_SPI_BUFFER_CELL };
    // Dotstar SPI, SCK, MOSI, and TX_DMA
    let mut dotstars =
        Apa102::<11>::new_from_pins(p.SPI6, p.PB3, p.PB5, p.BDMA_CH0, dotstar_spi_buf.into());
    dotstars.set_drv_str_all(32);

    loop {
        led1.set_high();
        led2.set_high();
        led3.set_high();
        dotstars.set_color_all(WHITE);
        dotstars.update().await;
        Timer::after_millis(1000).await;

        led1.set_low();
        led2.set_low();
        led3.set_low();
        dotstars.set_color_all(BLUE);
        dotstars.update().await;
        Timer::after_millis(1000).await;
    }
}
