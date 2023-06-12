#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

// use cortex_m::{peripheral::NVIC, delay::Delay};
use cortex_m::{peripheral::NVIC};
use cortex_m_rt::entry;

use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{
    adc::{Adc, AdcPin, InternalChannel, Temperature},
    pac::Interrupt,
    Peripherals,
    gpio::{Input, Level, Output, Pull, Speed},
    gpio::low_level::Pin,
    interrupt, adc::SampleTime,
};
use embassy_sync::{pubsub::{PubSubChannel, Publisher}, blocking_mutex::raw::NoopRawMutex};
use embassy_time::{Delay, Duration, Timer, Ticker};

use static_cell::StaticCell;

use ateam_kicker_board::pins::{HighVoltageReadPin, BatteryVoltageReadPin, ChargePin, RegulatorDonePin, RegulatorFaultPin, RedStatusLedPin, GreenStatusLedPin, KickPin};

// #[embassy_executor::task]
// async fn run_critical_section_task(p: Peripherals) {
//     let reg_done = Input::new(p.PB4, Pull::None);
//     let reg_fault = Input::new(p.PB5, Pull::None);

//     let mut reg_charge = Output::new(p.PB3, Level::Low, Speed::Medium);
//     let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
//     let mut status_led_red = Output::new(p.PA12, Level::Low, Speed::Medium);

//     status_led_green.set_high();
//     Timer::after(Duration::from_millis(500)).await;
//     status_led_green.set_low();
//     Timer::after(Duration::from_millis(500)).await;

//     reg_charge.set_high();
//     Timer::after(Duration::from_millis(100)).await;
//     reg_charge.set_low();

//     loop {
//         reg_charge.set_low();

//         if reg_done.is_low() {
//             status_led_green.set_high();
//         } else {
//             status_led_green.set_low();
//         }

//         if reg_fault.is_low() {
//             status_led_red.set_high();
//         } else {
//             status_led_red.set_low();
//         }
//     }
// }

// #[embassy_executor::task]
// async fn read_adc_samples(aps: PubSubChannel::<NoopRawMutex, AnalogValues, 1, 2, 1>) -> ! {
//     let sub = aps.subscriber().unwrap();
//     loop {
//         info!(sub.)
//     }
// }

#[embassy_executor::task]
async fn sample_adc(mut adc: Adc<'static, embassy_stm32::peripherals::ADC>, 
        mut hv_pin: HighVoltageReadPin, 
        mut batt_pin: BatteryVoltageReadPin, 
        mut reg_charge: ChargePin,
        mut reg_done: RegulatorDonePin,
        mut reg_fault: RegulatorFaultPin,
        mut status_led_red: RedStatusLedPin,
        mut status_led_green: GreenStatusLedPin,
        mut kick_pin: KickPin) -> ! {

    let mut ticker = Ticker::every(Duration::from_millis(1));

    let mut kick = Output::new(kick_pin, Level::Low, Speed::Medium);

    let reg_done = Input::new(reg_done, Pull::None);
    let reg_fault = Input::new(reg_fault, Pull::None);

    let mut reg_charge = Output::new(reg_charge, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(status_led_green, Level::Low, Speed::Medium);
    let mut status_led_red = Output::new(status_led_red, Level::Low, Speed::Medium);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(500)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(500)).await;

    reg_charge.set_high();
    Timer::after(Duration::from_millis(600)).await;
    let reg_done_stat = reg_done.is_low();
    let reg_fault_stat = reg_fault.is_low();
    reg_charge.set_low();

    let mut hv = adc.read(&mut hv_pin) as u32;
    let mut bv = adc.read(&mut batt_pin) as u32;
    info!("hv V: {}, batt mv: {}", (hv * 200) / 1000, bv);

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
        hv = adc.read(&mut hv_pin) as u32;
        // let bv = (adc.read(&mut batt_pin) as u32 * 10) / 12;
        bv = adc.read(&mut batt_pin) as u32;


        info!("hv V: {}, batt mv: {}", (hv * 200) / 1000, bv);

        reg_charge.set_low();
        kick.set_low();

        if reg_done_stat {
            status_led_green.set_high();
        } else {
            status_led_green.set_low();
        }

        if reg_fault_stat {
            status_led_red.set_high();
        } else {
            status_led_red.set_low();
        }

        ticker.next().await;
    }
}

// static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[derive(Clone, Copy)]
struct AnalogValues {
    high_voltage: u16,
    batt_voltage: u16,
    temp: u16
}

// #[interrupt]
// unsafe fn I2C1() {
//     EXECUTOR_HIGH.on_interrupt();
// }


#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    info!("kicker startup!");
    let mut nvic: NVIC = unsafe { mem::transmute(()) };

    // let aps = PubSubChannel::<NoopRawMutex, AnalogValues, 1, 2, 1>::new();


    // highest priorty, energy management
    // medium priority, ADC publisher
    // low priority uart handler
    // lowest prioiryt main update loop

    // High-priority executor: I2C1, priority level 6
    // unsafe { nvic.set_priority(Interrupt::I2C1, 6 << 4) };
    // let spawner = EXECUTOR_HIGH.start(Interrupt::I2C1);
    // unwrap!(spawner.spawn(run_critical_section_task(p)));

    let mut adc = Adc::new(p.ADC, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles71_5);
    // let mut temp_ch = adc.enable_temperature(&mut Delay);

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(sample_adc(adc, p.PA0, p.PA1, p.PB3, p.PB4, p.PB5, p.PA12, p.PA11, p.PB0)));
    });
}