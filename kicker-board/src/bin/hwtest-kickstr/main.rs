// Test script for different kicker charge times
// Change the values in the `durations` array to the
// number of microseconds you'd like to charge for each
// trial.
// Push the button to charge and then kick. Afterwards,
// the bot will wait for the next button press for the
// current duration.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m_rt::entry;

use embassy_executor::Executor;
use embassy_stm32::{
    adc::{Adc, SampleTime},
    gpio::{Input, Level, Output, Pull, Speed},
};
use embassy_time::{Duration, Timer, Ticker};

use static_cell::StaticCell;

use ateam_kicker_board::*;
use ateam_kicker_board::pins::*;

#[embassy_executor::task]
async fn run_kick(mut adc: Adc<'static, PowerRailAdc>, 
        mut hv_pin: PowerRail200vReadPin, 
        mut rail_12v0_pin: PowerRail12vReadPin,
        reg_charge: ChargePin,
        status_led_red: RedStatusLedPin,
        status_led_green: GreenStatusLedPin,
        usr_btn_pin: UserBtnPin,
        kick_pin: KickPin) -> ! {

    let mut ticker = Ticker::every(Duration::from_millis(1));

    let mut kick = Output::new(kick_pin, Level::Low, Speed::Medium);

    let mut reg_charge = Output::new(reg_charge, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(status_led_green, Level::Low, Speed::Medium);
    let mut status_led_red = Output::new(status_led_red, Level::Low, Speed::Medium);

    let usr_btn = Input::new(usr_btn_pin, Pull::None);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(500)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(500)).await;

    let mut vrefint = adc.enable_vrefint();
    let vrefint_sample = adc.read(&mut vrefint) as f32;

    let mut hv = adc.read(&mut hv_pin) as f32;
    let mut regv = adc.read(&mut rail_12v0_pin) as f32;
    info!("hv V: {}, 12v reg mv: {}", adc_200v_to_rail_voltage(adc_raw_to_v(hv, vrefint_sample)), adc_12v_to_rail_voltage(adc_raw_to_v(regv, vrefint_sample)));

    let start_up_battery_voltage = adc_v_to_battery_voltage(adc_raw_to_v(regv, vrefint_sample));
    if start_up_battery_voltage < 11.5 {
        status_led_red.set_high();
        warn!("regulator voltage is below 18.0 ({}), is the battery low or disconnected?", start_up_battery_voltage);
        warn!("refusing to continue");
        loop {
            reg_charge.set_low();

            kick.set_high();
            Timer::after(Duration::from_micros(500)).await;
            kick.set_low();
        
            Timer::after(Duration::from_millis(1000)).await;
        }
    }

    // in us
    let durations = [500, 1000, 2000, 4000];

    // For each duration, wait for button, charge, then kick
    for d in durations {
        'outer: while usr_btn.is_low() {
            while usr_btn.is_high() {
                defmt::info!("btn pressed! - initiating kick for {} us", d);
                break 'outer;
            }
        }

        Timer::after(Duration::from_millis(1000)).await;

        // We can't charge and kick at the same time...
        // The kicker fully charges within ~400 ms
        reg_charge.set_high();
        Timer::after(Duration::from_millis(450)).await;
        reg_charge.set_low();
    
        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.read(&mut vrefint) as f32;

        hv = adc.read(&mut hv_pin) as f32;
        regv = adc.read(&mut rail_12v0_pin) as f32;
        info!("hv V: {}, batt mv: {}", adc_200v_to_rail_voltage(adc_raw_to_v(hv, vrefint_sample)), adc_12v_to_rail_voltage(adc_raw_to_v(regv, vrefint_sample)));
    
        Timer::after(Duration::from_millis(1000)).await;

        // High = discharge into the solenoid
        kick.set_high();
        Timer::after(Duration::from_micros(d)).await;
        // Low = don't discharge into solenoid
        kick.set_low();

        Timer::after(Duration::from_millis(1000)).await;

    }

    loop {
        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.read(&mut vrefint) as f32;

        hv = adc.read(&mut hv_pin) as f32;
        regv = adc.read(&mut rail_12v0_pin) as f32;

        info!("hv V: {}, batt mv: {}", adc_200v_to_rail_voltage(adc_raw_to_v(hv, vrefint_sample)), adc_12v_to_rail_voltage(adc_raw_to_v(regv, vrefint_sample)));

        reg_charge.set_low();
        kick.set_low();

        ticker.next().await;
    }
}

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    info!("kicker startup!");

    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES480);

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(run_kick(adc, p.PC0, p.PC1, p.PE4, p.PE1, p.PE0, p.PD4, p.PE5)));
    });
}