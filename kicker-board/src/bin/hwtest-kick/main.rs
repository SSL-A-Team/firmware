#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use tasks::get_system_config;
use {defmt_rtt as _, panic_probe as _};

use cortex_m_rt::entry;

use embassy_executor::Executor;
use embassy_stm32::{
    adc::{Adc, SampleTime},
    gpio::{Input, Level, Output, Pull, Speed},
    opamp::{OpAmp, OpAmpGain, OpAmpSpeed},
};
use embassy_time::{Duration, Ticker, Timer};

use static_cell::StaticCell;

use ateam_kicker_board::pins::*;
use ateam_kicker_board::*;

#[embassy_executor::task]
async fn run_kick(
    mut adc: Adc<'static, PowerRailAdc>,
    mut hv_pin: PowerRail200vReadPin,
    mut rail_12v0_pin: PowerRailVswReadPin,
    reg_charge: ChargePin,
    status_led_red: RedStatusLedPin,
    status_led_green: GreenStatusLedPin,
    usr_btn_pin: UserBtnPin,
    // chip_pin: ChipPin,
    kick_pin: KickPin,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(1));

    let mut kick = Output::new(kick_pin, Level::Low, Speed::Medium);

    let mut reg_charge = Output::new(reg_charge, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(status_led_green, Level::Low, Speed::Medium);
    let _status_led_red = Output::new(status_led_red, Level::Low, Speed::Medium);

    let usr_btn = Input::new(usr_btn_pin, Pull::None);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(500)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(500)).await;

    let mut vrefint = adc.enable_vrefint();
    let vrefint_sample = adc.blocking_read(&mut vrefint) as f32;

    let mut hv = adc.blocking_read(&mut hv_pin) as f32;
    let mut regv = adc.blocking_read(&mut rail_12v0_pin) as f32;
    info!(
        "hv V: {}, 12v reg mv: {}",
        adc_200v_to_rail_voltage(adc_raw_to_v(hv, vrefint_sample)),
        adc_12v_to_rail_voltage(adc_raw_to_v(regv, vrefint_sample))
    );

    let _start_up_battery_voltage = adc_v_to_battery_voltage(adc_raw_to_v(regv, vrefint_sample));
    // if start_up_battery_voltage < 11.5 {
    //     status_led_red.set_high();
    //     warn!("regulator voltage is below 18.0 ({}), is the battery low or disconnected?", start_up_battery_voltage);
    //     warn!("refusing to continue");
    //     loop {
    //         reg_charge.set_low();

    //         kick.set_high();
    //         Timer::after(Duration::from_micros(500)).await;
    //         kick.set_low();

    //         Timer::after(Duration::from_millis(1000)).await;
    //     }
    // }

    'outer: while usr_btn.is_low() {
        while usr_btn.is_high() {
            defmt::info!("btn pressed! - initiating kick cycle");
            break 'outer;
        }
    }

    Timer::after(Duration::from_millis(1000)).await;

    reg_charge.set_high();
    Timer::after(Duration::from_millis(1700)).await;
    reg_charge.set_low();

    let mut vrefint = adc.enable_vrefint();
    let vrefint_sample = adc.blocking_read(&mut vrefint) as f32;

    hv = adc.blocking_read(&mut hv_pin) as f32;
    regv = adc.blocking_read(&mut rail_12v0_pin) as f32;
    info!(
        "hv V: {}, batt mv: {}",
        adc_200v_to_rail_voltage(adc_raw_to_v(hv, vrefint_sample)),
        adc_12v_to_rail_voltage(adc_raw_to_v(regv, vrefint_sample))
    );

    Timer::after(Duration::from_millis(1000)).await;

    kick.set_high();
    Timer::after(Duration::from_micros(10000)).await;
    kick.set_low();

    Timer::after(Duration::from_millis(2000)).await;

    kick.set_high();
    Timer::after(Duration::from_micros(500000)).await;
    kick.set_low();

    Timer::after(Duration::from_millis(1000)).await;

    loop {
        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.blocking_read(&mut vrefint) as f32;

        hv = adc.blocking_read(&mut hv_pin) as f32;
        regv = adc.blocking_read(&mut rail_12v0_pin) as f32;

        info!(
            "hv V: {}, batt mv: {}",
            adc_200v_to_rail_voltage(adc_raw_to_v(hv, vrefint_sample)),
            adc_12v_to_rail_voltage(adc_raw_to_v(regv, vrefint_sample))
        );

        reg_charge.set_low();
        kick.set_low();

        ticker.next().await;
    }
}

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let stm32_config = get_system_config(tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(stm32_config);

    info!("kicker startup!");

    let _vsw_en = Output::new(p.PE10, Level::High, Speed::Medium);

    let mut hv_opamp_inst = OpAmp::new(p.OPAMP3, OpAmpSpeed::HighSpeed);
    let _hv_opamp = hv_opamp_inst.buffer_ext(p.PB0, p.PB1, OpAmpGain::Mul2);

    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES247_5);

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(run_kick(
            adc, p.PC3, p.PA1, p.PB15, p.PE0, p.PB9, p.PB5, p.PD9
        )));
    });
}
