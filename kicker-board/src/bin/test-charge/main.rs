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

use ateam_kicker_board::{adc_publisher::{AdcPublisher, SamplePin}, pins::{HighVoltageReadPin, BatteryVoltageReadPin}};

#[embassy_executor::task]
async fn run_critical_section_task(p: Peripherals) {
    let reg_done = Input::new(p.PB4, Pull::None);
    let reg_fault = Input::new(p.PB5, Pull::None);

    let mut reg_charge = Output::new(p.PB3, Level::Low, Speed::Medium);
    let mut status_led_green = Output::new(p.PA11, Level::Low, Speed::Medium);
    let mut status_led_red = Output::new(p.PA12, Level::Low, Speed::Medium);

    status_led_green.set_high();
    Timer::after(Duration::from_millis(500)).await;
    status_led_green.set_low();
    Timer::after(Duration::from_millis(500)).await;

    reg_charge.set_high();
    Timer::after(Duration::from_millis(100)).await;
    reg_charge.set_low();

    loop {
        reg_charge.set_low();

        if reg_done.is_low() {
            status_led_green.set_high();
        } else {
            status_led_green.set_low();
        }

        if reg_fault.is_low() {
            status_led_red.set_high();
        } else {
            status_led_red.set_low();
        }
    }
}

#[embassy_executor::task]
async fn read_adc_samples(aps: PubSubChannel::<NoopRawMutex, AnalogValues, 1, 2, 1>) -> ! {
    let sub = aps.subscriber().unwrap();
    loop {
        info!(sub.)
    }
}

#[embassy_executor::task]
async fn sample_adc(mut adc: Adc<'static, embassy_stm32::peripherals::ADC>, 
        mut temp_ch: Temperature, 
        mut hv_pin: HighVoltageReadPin, 
        mut batt_pin: BatteryVoltageReadPin, 
        aps: PubSubChannel::<NoopRawMutex, AnalogValues, 1, 2, 1>) -> ! {

    let psh = aps.publisher().unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut av = AnalogValues {
        high_voltage: 0, 
        batt_voltage: 0,
        temp: 0 
    };
    loop {
        av.high_voltage = adc.read(&mut hv_pin);
        av.batt_voltage = adc.read(&mut batt_pin);
        av.temp = adc.read_internal(&mut temp_ch);

        psh.publish_immediate(av);

        // Timer::after(Duration::from_millis(1));
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

    let aps = PubSubChannel::<NoopRawMutex, AnalogValues, 1, 2, 1>::new();


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
    let mut temp_ch = adc.enable_temperature(&mut Delay);

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(sample_adc(adc, temp_ch, p.PA0, p.PA1, aps)));
    });
}