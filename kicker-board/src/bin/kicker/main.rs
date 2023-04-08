#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;

use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{
    adc::{Adc, AdcPin},
    pac::Interrupt,
    Peripherals,
    gpio::{Input, Level, Output, Pull, Speed},
    gpio::low_level::Pin,
    interrupt,
};
use embassy_time::{Delay, Duration, Timer};

use static_cell::StaticCell;

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
async fn low_pri_ticker() {
    loop {
        info!("[low] Starting long computation");

        Timer::after(Duration::from_ticks(32983)).await;
    }
}

// static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

// #[interrupt]
// unsafe fn I2C1() {
//     EXECUTOR_HIGH.on_interrupt();
// }

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    info!("kicker startup!");
    let mut nvic: NVIC = unsafe { mem::transmute(()) };

    // highest priorty, energy management
    // medium priority, ADC publisher
    // low priority uart handler
    // lowest prioiryt main update loop

    // High-priority executor: I2C1, priority level 6
    // unsafe { nvic.set_priority(Interrupt::I2C1, 6 << 4) };
    // let spawner = EXECUTOR_HIGH.start(Interrupt::I2C1);
    // unwrap!(spawner.spawn(run_critical_section_task(p)));

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(low_pri_ticker()));
    });
}