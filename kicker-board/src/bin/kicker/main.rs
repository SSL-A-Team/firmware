#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem;
use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;

use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    pac::Interrupt,
    gpio::{Level, Output, Speed},
    interrupt,
};
use embassy_time::{Delay};

use ateam_kicker_board::{
    adc_raw_to_mv,
    adc_mv_to_battery_voltage,
    adc_mv_to_rail_voltage,
    kick_manager::{
        KickManager, 
        KickType},
    pins::{HighVoltageReadPin,
        BatteryVoltageReadPin,
        ChargePin,
        KickPin,
        ChipPin}
};

#[embassy_executor::task]
async fn high_pri_kick_task(
        mut adc: Adc<'static, embassy_stm32::peripherals::ADC>,
        charge_pin: ChargePin,
        kick_pin: KickPin,
        chip_pin: ChipPin,
        mut rail_pin: HighVoltageReadPin,
        mut battery_voltage_pin: BatteryVoltageReadPin) -> ! {

    let charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);
    let mut kick_manager = KickManager::new(charge_pin, kick_pin, chip_pin);

    loop {
        let rail_voltage = adc_mv_to_rail_voltage(adc_raw_to_mv(adc.read(&mut rail_pin) as f32));
        let battery_voltage = adc_mv_to_battery_voltage(adc_raw_to_mv(adc.read(&mut battery_voltage_pin) as f32));

        // read breakbeam
        // read commands

        // convert breakbeam state + command into action(s)

        // TODO use result to send errors upstream
        let _ = kick_manager.command(battery_voltage, rail_voltage, false, KickType::None, 0.0).await;

        // publish adc values
        // publish breakbeam values
        // do ticker
    }

}

#[embassy_executor::task]
async fn low_pri_coms_task() {
    loop {
        // read packets from control
        // publish command to kick channel

        // read packets from kick system channels (voltages, breakbeam, status)
        // update LEDs
        // send upstream packet
    }
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn I2C1() {
    EXECUTOR_HIGH.on_interrupt();
}

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    info!("kicker startup!");
    let mut nvic: NVIC = unsafe { mem::transmute(()) };

    let mut adc = Adc::new(p.ADC, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles71_5);



    // high priority executor handles kicking system
    // High-priority executor: I2C1, priority level 6
    unsafe { nvic.set_priority(Interrupt::I2C1, 6 << 4) };
    let spawner = EXECUTOR_HIGH.start(Interrupt::I2C1);
    unwrap!(spawner.spawn(high_pri_kick_task(adc, p.PB3, p.PB0, p.PB1, p.PA0, p.PA1)));

    // low priority executor handles coms and user IO
    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(low_pri_coms_task()));
    });
}