#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(sync_unsafe_cell)]

use ateam_kicker_board::{drivers::breakbeam::Breakbeam, pins::{BreakbeamLeftAgpioPin, BreakbeamRightAgpioPin}, tasks::get_system_config};
use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use libm::{fmaxf, fminf};

use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    bind_interrupts,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt,
    pac::Interrupt,
    peripherals,
};
use embassy_time::{Duration, Instant, Ticker, Timer};

use ateam_kicker_board::{
    adc_raw_to_v, adc_200v_to_rail_voltage,
    kick_manager::{KickManager, KickType},
    pins::{
        BlueStatusLed1Pin, BlueStatusLed2Pin, ChargePin, ChipPin,
        PowerRail200vReadPin, KickPin, RedStatusLedPin,
    },
};


use ateam_common_packets::bindings_kicker::{
    KickRequest::{self, KR_ARM, KR_DISABLE},
    KickerControl, KickerTelemetry,
};

const MAX_TX_PACKET_SIZE: usize = 16;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 3;

const MAX_KICK_SPEED: f32 = 5.5;
const SHUTDOWN_KICK_DUTY: f32 = 0.20;

pub const CHARGE_TARGET_VOLTAGE: f32 = 182.0;
pub const CHARGE_OVERVOLT_THRESH_VOLTAGE: f32 = 190.0;
pub const CHARGED_THRESH_VOLTAGE: f32 = 170.0;
pub const CHARGE_SAFE_VOLTAGE: f32 = 5.0;

fn get_empty_control_packet() -> KickerControl {
    KickerControl {
        _bitfield_align_1: [],
        _bitfield_1: KickerControl::new_bitfield_1(0, 0, 0),
        kick_request: KickRequest::KR_DISABLE,
        kick_speed: 0.0,
    }
}

fn get_empty_telem_packet() -> KickerTelemetry {
    KickerTelemetry {
        _bitfield_align_1: [],
        _bitfield_1: KickerTelemetry::new_bitfield_1(0, 0, 0, 0),
        rail_voltage: 0.0,
        battery_voltage: 0.0,
    }
}

#[embassy_executor::task]
async fn high_pri_kick_task(
    mut ball_detected_output: Output<'static>,
    kick_commanded_input: Input<'static>,
    mut adc: Adc<'static, embassy_stm32::peripherals::ADC1>,
    charge_pin: ChargePin,
    kick_pin: KickPin,
    chip_pin: ChipPin,
    breakbeam_tx: BreakbeamLeftAgpioPin,
    breakbeam_rx: BreakbeamRightAgpioPin,
    mut rail_pin: PowerRail200vReadPin,
    err_led_pin: RedStatusLedPin,
    ball_detected_led1_pin: BlueStatusLed1Pin,
    ball_detected_led2_pin: BlueStatusLed2Pin,
) -> ! {
    // pins/safety management
    let charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);
    let mut kick_manager = KickManager::new(charge_pin, kick_pin, chip_pin);

    // debug LEDs
    let mut err_led = Output::new(err_led_pin, Level::Low, Speed::Low);
    let mut ball_detected_led1 = Output::new(ball_detected_led1_pin, Level::Low, Speed::Low);
    let mut ball_detected_led2 = Output::new(ball_detected_led2_pin, Level::Low, Speed::Low);

    // TODO dotstars

    let mut breakbeam = Breakbeam::new(breakbeam_tx, breakbeam_rx);

    // coms buffers
    let mut telemetry_enabled: bool; //  = false;
    let mut kicker_control_packet: KickerControl = get_empty_control_packet();

    // bookkeeping for latched state
    let mut kick_command_cleared: bool = false;
    let mut latched_command = KickRequest::KR_DISABLE;
    let mut error_latched: bool = false;
    let mut charge_hv_rail: bool = false;

    // power down status
    let mut shutdown_requested: bool = false;
    let mut shutdown_completed: bool = false;

    // loop rate control
    let mut ticker = Ticker::every(Duration::from_millis(1));

    breakbeam.enable_tx();
    loop {
        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.read(&mut vrefint);

        let rail_voltage = adc_200v_to_rail_voltage(adc_raw_to_v(adc.read(&mut rail_pin) as f32, vrefint_sample as f32));
        // let battery_voltage =
        //     adc_v_to_battery_voltage(adc_raw_to_v(adc.read(&mut battery_voltage_pin) as f32, vrefint_sample as f32));
        // optionally pre-flag errors?
        let battery_voltage = 22.5;

        /////////////////////////////////////
        //  process any available packets  //
        /////////////////////////////////////

        let kick_commanded = kick_commanded_input.is_high();

        // TODO: read breakbeam
        let ball_detected = breakbeam.read();

        ///////////////////////////////////////////////
        //  manage repetitive kick commands + state  //
        ///////////////////////////////////////////////
        
        let kick_command = 

        kick_manager.command(battery_voltage, rail_voltage, charge_hv_rail, kick_command, kick_strength).await;

        // TODO write ball detected GPIO

        // LEDs
        if error_latched {
            err_led.set_high();
        } else {
            err_led.set_low();
        }

        if ball_detected {
            ball_detected_led1.set_high();
            ball_detected_led2.set_high();

        } else {
            ball_detected_led1.set_low();
            ball_detected_led2.set_low();
        }
        // TODO Dotstar

        // loop rate control @1KHz
        ticker.next().await;
    }
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn TIM2() {
    EXECUTOR_HIGH.on_interrupt();
}

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let sys_cfg = get_system_config(ateam_kicker_board::tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_cfg);

    info!("kicker startup!");

    let _status_led = Output::new(p.PA11, Level::High, Speed::Low);

    let ball_detected_output = Output::new(p.PA1, Level::Low, Speed::Low);
    let kick_commanded_input = Input::new(p.PA1, Pull::Down);

    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES480);

    // high priority executor handles kicking system
    // High-priority executor: I2C1, priority level 6
    // TODO CHECK THIS IS THE HIGHEST PRIORITY
    embassy_stm32::interrupt::TIM2.set_priority(embassy_stm32::interrupt::Priority::P6);
    let spawner = EXECUTOR_HIGH.start(Interrupt::TIM2);
    unwrap!(spawner.spawn(high_pri_kick_task(kick_commanded_input, ball_detected_output, adc, p.PE4, p.PE5, p.PE6, p.PA1, p.PA0, p.PC0, p.PE1, p.PE2, p.PE3)));

    loop {
        Timer::after_millis(1000).await;
    }
}
