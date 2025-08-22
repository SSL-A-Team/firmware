#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    spi::{Config, Spi},
    time::Hertz,
};

use ateam_lib_stm32::drivers::flash::at25df041b::AT25DF041B;

use ateam_kicker_board::{tasks::get_system_config, *};

use panic_probe as _;
// use panic_halt as _;

#[link_section = ".bss"]
static FLASH_RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
#[link_section = ".bss"]
static FLASH_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let sys_config = get_system_config(tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_config);

    info!("kicker startup!");

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let spi = Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH4, p.DMA1_CH3, spi_config,
    );

    let rx_buf = FLASH_RX_BUF.init([0; 256]);
    let tx_buf = FLASH_TX_BUF.init([0; 256]);

    let mut flash: AT25DF041B<'static, true> = AT25DF041B::new(spi, p.PB12, rx_buf, tx_buf);
    let res = flash.verify_chip_id().await;
    if res.is_err() {
        defmt::error!("failed to verify flash chip ID");
    } else {
        defmt::info!("verified flash chip ID");
    }

    loop {}
}
