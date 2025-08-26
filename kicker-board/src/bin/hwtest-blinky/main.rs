#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, SampleTime},
    gpio::{Input, Level, Output, Pull, Speed},
    spi::{Config, Spi},
    time::mhz,
    Peri,
};
use embassy_time::{Duration, Timer};

use static_cell::StaticCell;

use ateam_kicker_board::pins::*;
use ateam_kicker_board::{tasks::get_system_config, *};

use panic_probe as _;
// use panic_halt as _;

#[embassy_executor::task]
async fn blink(
    reg_charge: Peri<'static, ChargePin>,
    status_led_red: Peri<'static, RedStatusLedPin>,
    status_led_green: Peri<'static, GreenStatusLedPin>,
    status_led_blue1: Peri<'static, BlueStatusLedPin>,
    usr_btn_pin: Peri<'static, UserBtnPin>,
    mut adc: Adc<'static, PowerRailAdc>,
    mut rail_200v_pin: Peri<'static, PowerRail200vReadPin>,
    mut rail_12v0_pin: Peri<'static, PowerRailVswReadPin>,
    mut rail_5v0_pin: Peri<'static, PowerRail5v0ReadPin>,
    mut rail_3v3_pin: Peri<'static, PowerRail3v3ReadPin>,
) -> ! {
    let mut reg_charge = Output::new(reg_charge, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(status_led_green, Level::High, Speed::Medium);
    let mut status_led_red = Output::new(status_led_red, Level::Low, Speed::Medium);
    let mut status_led_blue1 = Output::new(status_led_blue1, Level::Low, Speed::Medium);

    let usr_btn = Input::new(usr_btn_pin, Pull::None);

    // let mut temp = adc.enable_temperature();
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES247_5);

    'outer: while usr_btn.is_low() {
        while usr_btn.is_high() {
            defmt::info!("btn pressed! - cycle");
            break 'outer;
        }
    }

    Timer::after(Duration::from_millis(1000)).await;

    loop {
        reg_charge.set_low();

        status_led_green.set_high();
        status_led_blue1.set_high();
        status_led_red.set_low();
        Timer::after(Duration::from_millis(500)).await;

        status_led_green.set_low();
        status_led_blue1.set_low();
        status_led_red.set_high();
        Timer::after(Duration::from_millis(500)).await;

        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.blocking_read(&mut vrefint) as f32;

        let raw_200v = adc.blocking_read(&mut rail_200v_pin) as f32;
        let raw_12v = adc.blocking_read(&mut rail_12v0_pin) as f32;
        let raw_5v0 = adc.blocking_read(&mut rail_5v0_pin) as f32;
        let raw_3v3 = adc.blocking_read(&mut rail_3v3_pin) as f32;

        // defmt::info!("voltages - 200v ({}), Vsw ({}), 5v0 ({}), 3v3 ({})",
        // adc_200v_to_rail_voltage(raw_200v),
        // adc_12v_to_rail_voltage(raw_12v),
        // adc_5v0_to_rail_voltage(raw_5v0),
        // adc_3v3_to_rail_voltage(raw_3v3));
        defmt::info!(
            "voltages - 200v ({}), Vsw ({}), 5v0 ({}), 3v3 ({})",
            adc_200v_to_rail_voltage(adc_raw_to_v(raw_200v, vrefint_sample)),
            adc_12v_to_rail_voltage(adc_raw_to_v(raw_12v, vrefint_sample)),
            adc_5v0_to_rail_voltage(adc_raw_to_v(raw_5v0, vrefint_sample)),
            adc_3v3_to_rail_voltage(adc_raw_to_v(raw_3v3, vrefint_sample))
        );
    }
}

#[embassy_executor::task]
async fn dotstar_lerp_task(
    dotstar_spi: Peri<'static, DotstarSpi>,
    dotstar_mosi_pin: Peri<'static, DotstarSpiMosiPin>,
    dotstar_sck_pin: Peri<'static, DotstarSpiSckPin>,
    dotstar_tx_dma: Peri<'static, DotstarSpiTxDma>,
) {
    let mut dotstar_spi_config = Config::default();
    dotstar_spi_config.frequency = mhz(1);

    let mut dotstar_spi = Spi::new_txonly(
        dotstar_spi,
        dotstar_sck_pin,
        dotstar_mosi_pin,
        dotstar_tx_dma,
        dotstar_spi_config,
    );

    // let mut dotstar = Apa102::new(dotstar_spi);

    // dotstar.write([RGB8 {r: 10, g: 10, b: 10}, RGB8 {r: 10, g: 10, b: 10}]);

    let mut counting_up: bool = true;
    let mut val = 0;
    loop {
        // let _ = dotstar.write([RGB8 { r: val, g: val, b: val }, RGB8 { r: val / 2, g: val / 2, b: val / 2 }].iter().cloned());
        let _ = dotstar_spi
            .write(&[
                0x00 as u8, 0x00, 0x00, 0x00, 0xE7, val, 0x00, val, 0xE7, val, 0x00, val, 0xFF,
                0xFF, 0xFF, 0xFF,
            ])
            .await;

        if counting_up {
            val += 1;
        } else {
            val -= 1;
        }

        if val == 255 {
            counting_up = false;
        } else if val == 0 {
            counting_up = true;
        }

        Timer::after_millis(5).await;
    }
}

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let sys_config = get_system_config(tasks::ClkSource::External8MHzOscillator);
    let p = embassy_stm32::init(sys_config);

    info!("kicker startup!");

    // let _kick_pin = Output::new(p.PE5, Level::Low, Speed::Medium);
    // let _chip_pin = Output::new(p.PE6, Level::Low, Speed::Medium);

    let _vsw_en = Output::new(p.PE10, Level::High, Speed::Medium);

    info!("kicker startup 1.5!");

    let adc = Adc::new(p.ADC1);

    info!("kicker startup 2!");

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        // unwrap!(spawner.spawn(shutdown_int(p.PD5, p.EXTI5, p.PD6)));
        unwrap!(spawner.spawn(blink(
            p.PB15, p.PE0, p.PB9, p.PE1, p.PB5, adc, p.PC3, p.PA1, p.PA2, p.PA3
        )));
        unwrap!(spawner.spawn(dotstar_lerp_task(p.SPI4, p.PE6, p.PE2, p.DMA2_CH8)));
    });
}
