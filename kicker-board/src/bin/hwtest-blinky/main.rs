#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_executor::Executor;
use embassy_stm32::{
    adc::{Adc, SampleTime}, 
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed}, 
    spi::{Config, Spi}, time::mhz,
};
use embassy_time::{Duration, Timer};

use static_cell::StaticCell;

use smart_leds::{SmartLedsWrite, RGB8};
use apa102_spi::Apa102;

use ateam_kicker_board::{tasks::get_system_config, *};
use ateam_kicker_board::pins::*;

use panic_probe as _;
// use panic_halt as _;

#[embassy_executor::task]
async fn blink( 
        reg_charge: ChargePin,
        status_led_red: RedStatusLedPin,
        status_led_green: GreenStatusLedPin,
        status_led_blue1: BlueStatusLed1Pin,
        status_led_blue2: BlueStatusLed2Pin,
        usr_btn_pin: UserBtnPin,
        mut adc: Adc<'static, PowerRailAdc>,
        mut rail_200v_pin: PowerRail200vReadPin,
        mut rail_12v0_pin: PowerRail12vReadPin,
        mut rail_6v2_pin: PowerRail6v2ReadPin,
        mut rail_5v0_pin: PowerRail5v0ReadPin) -> ! {

    let mut reg_charge = Output::new(reg_charge, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(status_led_green, Level::High, Speed::Medium);
    let mut status_led_red = Output::new(status_led_red, Level::Low, Speed::Medium);
    let mut status_led_blue1 = Output::new(status_led_blue1, Level::Low, Speed::Medium);
    let mut status_led_blue2 = Output::new(status_led_blue2, Level::Low, Speed::Medium);

    let usr_btn = Input::new(usr_btn_pin, Pull::None);

    // let mut temp = adc.enable_temperature();
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES480);

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
        status_led_blue2.set_high();
        status_led_red.set_low();
        Timer::after(Duration::from_millis(500)).await;

        status_led_green.set_low();
        status_led_blue1.set_low();
        status_led_blue2.set_low();
        status_led_red.set_high();
        Timer::after(Duration::from_millis(500)).await;

        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.read(&mut vrefint) as f32;

        let raw_200v = adc.read(&mut rail_200v_pin) as f32;
        let raw_12v = adc.read(&mut rail_12v0_pin) as f32;
        let raw_6v2 = adc.read(&mut rail_6v2_pin) as f32;
        let raw_5v0 = adc.read(&mut rail_5v0_pin) as f32;
        let raw_int = adc.read(&mut vrefint) as f32;

        defmt::info!("voltages - 200v ({}), 12v0 ({}), 6v2 ({}), 5v0 ({}), 3v3 ({})",
        adc_200v_to_rail_voltage(adc_raw_to_v(raw_200v, vrefint_sample)),
        adc_12v_to_rail_voltage(adc_raw_to_v(raw_12v, vrefint_sample)),
        adc_6v2_to_rail_voltage(adc_raw_to_v(raw_6v2, vrefint_sample)),
        adc_5v0_to_rail_voltage(adc_raw_to_v(raw_5v0, vrefint_sample)),
        adc_3v3_to_rail_voltage(adc_raw_to_v(raw_int, vrefint_sample)));


    }
}

#[embassy_executor::task]
async fn dotstar_lerp_task(dotstar_spi: DotstarSpi,
    dotstar_sck_pin: DotstarSpiSckPin,
    dotstar_mosi_pin: DotstarSpiMosiPin,
    dotstar_tx_dma: DotstarSpiTxDma) {

    let mut dotstar_spi_config = Config::default();
    dotstar_spi_config.frequency = mhz(1);

    let mut dotstar_spi = Spi::new_txonly(
        dotstar_spi,
        dotstar_sck_pin,
        dotstar_mosi_pin,
        dotstar_tx_dma,
        dotstar_spi_config);

    // let mut dotstar = Apa102::new(dotstar_spi);

    // dotstar.write([RGB8 {r: 10, g: 10, b: 10}, RGB8 {r: 10, g: 10, b: 10}]);

    let mut counting_up: bool = true;
    let mut val = 0;
    loop {
        // let _ = dotstar.write([RGB8 { r: val, g: val, b: val }, RGB8 { r: val / 2, g: val / 2, b: val / 2 }].iter().cloned());
        let _ = dotstar_spi.write(&[0x00 as u8, 0x00, 0x00, 0x00, 0xE7, val, 0x00, val, 0xE7, val, 0x00, val, 0xFF, 0xFF, 0xFF, 0xFF]).await;
        
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

#[embassy_executor::task]
async fn shutdown_int(pwr_btn_int_pin: PowerBtnIntPin,
                      pwr_btn_int_exti: PowerBtnIntExti,
                      pwr_kill_pin: PowerKillPin) {

    let mut pwr_btn_int = ExtiInput::new(pwr_btn_int_pin, pwr_btn_int_exti, Pull::Down);
    let mut pwr_kill = Output::new(pwr_kill_pin, Level::High, Speed::Medium);

    defmt::info!("task waiting for btn int from power front end...");

    pwr_btn_int.wait_for_rising_edge().await;

    defmt::warn!("received request to power down.");

    Timer::after(Duration::from_millis(1000)).await;

    // TODO do anything you need to do

    pwr_kill.set_low();

    loop {}
}

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let sys_config = get_system_config(tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_config);

    info!("kicker startup!");

    let _kick_pin = Output::new(p.PE5, Level::Low, Speed::Medium);
    let _chip_pin = Output::new(p.PE6, Level::Low, Speed::Medium);
    
    let adc = Adc::new(p.ADC1);

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        // unwrap!(spawner.spawn(shutdown_int(p.PD5, p.EXTI5, p.PD6)));
        unwrap!(spawner.spawn(blink(p.PE4, p.PE1, p.PE0, p.PE2, p.PE3, p.PD4, adc, p.PC0, p.PC1, p.PC3, p.PC2)));
        unwrap!(spawner.spawn(dotstar_lerp_task(p.SPI1, p.PA5, p.PA7, p.DMA2_CH3)));
    });
}