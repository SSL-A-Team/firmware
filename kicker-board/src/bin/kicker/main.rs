#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use core::mem;
use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;

use libm::{fminf, fmaxf};

use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    pac::Interrupt,
    gpio::{Level, Output, Speed},
    interrupt,
    usart::{Uart, Parity, StopBits, Config}
};
use embassy_time::{Delay, Duration, Instant, Ticker};

use ateam_kicker_board::{
    adc_raw_to_mv,
    adc_mv_to_battery_voltage,
    adc_mv_to_rail_voltage,
    kick_manager::{
        KickManager, 
        KickType},
    pins::{
        HighVoltageReadPin, BatteryVoltageReadPin,
        ChargePin, KickPin, ChipPin,
        ComsUartModule, ComsUartRxDma, ComsUartTxDma, RedStatusLedPin, BlueStatusLedPin},
    queue::Buffer,
    uart_queue::{
        UartReadQueue,
        UartWriteQueue,
    }
};

use ateam_common_packets::bindings_kicker::{KickerControl, KickerTelemetry, KickRequest::{self, KR_ARM, KR_DISABLE}};

const MAX_TX_PACKET_SIZE: usize = 16;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 3;

const MAX_KICK_SPEED: f32 = 5.5;
const SHUTDOWN_KICK_DUTY: f32 = 0.05;

pub const CHARGE_TARGET_VOLTAGE: f32 = 182.0;
pub const CHARGE_OVERVOLT_THRESH_VOLTAGE: f32 = 190.0;
pub const CHARGED_THRESH_VOLTAGE: f32 = 170.0;
pub const CHARGE_SAFE_VOLTAGE: f32 = 5.0;

// control communications tx buffer
// #[link_section = ".axisram.buffers"]
static mut COMS_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static COMS_QUEUE_TX: UartWriteQueue<ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut COMS_BUFFERS_TX });

// control communications rx buffer
// #[link_section = ".axisram.buffers"]
static mut COMS_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static COMS_QUEUE_RX: UartReadQueue<ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut COMS_BUFFERS_RX });

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
        coms_reader: &'static UartReadQueue<'static, ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
        coms_writer: &'static UartWriteQueue<'static, ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
        mut adc: Adc<'static, embassy_stm32::peripherals::ADC>,
        charge_pin: ChargePin,
        kick_pin: KickPin,
        chip_pin: ChipPin,
        mut rail_pin: HighVoltageReadPin,
        mut battery_voltage_pin: BatteryVoltageReadPin,
        err_led_pin: RedStatusLedPin,
        ball_detected_led_pin: BlueStatusLedPin) -> ! {

    // pins/safety management
    let charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);
    let mut kick_manager = KickManager::new(charge_pin, kick_pin, chip_pin);

    // debug LEDs
    let mut err_led = Output::new(err_led_pin, Level::Low, Speed::Low);
    let mut ball_detected_led = Output::new(ball_detected_led_pin, Level::Low, Speed::Low);
    // TODO dotstars

    // coms buffers
    let mut telemetry_enabled: bool; //  = false;
    let mut kicker_control_packet: KickerControl = get_empty_control_packet();
    let mut kicker_telemetry_packet: KickerTelemetry = get_empty_telem_packet();

    // bookkeeping for latched state
    let mut kick_command_cleared: bool = false;
    let mut latched_command = KickRequest::KR_DISABLE;
    let mut error_latched: bool = false;

    // power down status
    let mut shutdown_requested: bool = false;
    let mut shutdown_completed: bool = false;

    // loop rate control
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last_packet_sent_time = Instant::now();

    loop {
        let rail_voltage = adc_mv_to_rail_voltage(adc_raw_to_mv(adc.read(&mut rail_pin) as f32));
        let battery_voltage = adc_mv_to_battery_voltage(adc_raw_to_mv(adc.read(&mut battery_voltage_pin) as f32));
        // optionally pre-flag errors? 

        /////////////////////////////////////
        //  process any available packets  //
        /////////////////////////////////////
        
        while let Ok(res) = coms_reader.try_dequeue() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<KickerControl>() {
                defmt::warn!("got invalid packet of len {:?} data: {:?}", buf.len(), buf);
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // copy receieved uart bytes into packet
                let state = &mut kicker_control_packet as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<KickerControl>() {
                    *state.offset(i as isize) = buf[i];
                }                
            }
        }

        // TODO: read breakbeam
        let ball_detected = false;

        ///////////////////////////////////////////////
        //  manage repetitive kick commands + state  //
        ///////////////////////////////////////////////

        // update telemetry requests
        telemetry_enabled = kicker_control_packet.telemetry_enabled() != 0;

        // for now shutdown requests will be latched and a reboot is required to re-power
        if kicker_control_packet.request_power_down() != 0 {
            shutdown_requested = true;
        }

        // check if we've met the criteria for completed shutdown
        if shutdown_requested && rail_voltage < CHARGE_SAFE_VOLTAGE {
            shutdown_completed = true;
        }

        if rail_voltage > CHARGE_OVERVOLT_THRESH_VOLTAGE {
            error_latched = true;
        }

        // charge if were not in shutdown mode AND the control board has requested any state but "DISABLE" 
        let charge_hv_rail = if shutdown_requested || shutdown_completed {
            false
        } else {
            kicker_control_packet.kick_request != KickRequest::KR_DISABLE
        };

        // scale kick strength from m/s to duty for the critical section
        // if shutdown is requested and not complete, set kick discharge kick strength to 5%
        let kick_strength = if shutdown_completed {
            0.0
        }else if shutdown_requested { 
            SHUTDOWN_KICK_DUTY
        } else {
            fmaxf(0.0, fminf(MAX_KICK_SPEED, kicker_control_packet.kick_speed)) / MAX_KICK_SPEED 
        };

        // if control requests only an ARM or DISABLE, clear the active command
        // software/joystick is not asserting a kick event, so any future kick event is a unique request
        if kicker_control_packet.kick_request == KR_ARM || kicker_control_packet.kick_request == KR_DISABLE {
            kick_command_cleared = true;
        }

        // if the previous command has cleared acknowledgement, then any new request is a unique event and can be latched
        if kick_command_cleared {
            latched_command = kicker_control_packet.kick_request;
        }

        // determine if the latched command is actionable
        // if a shutdown is requested and not completed, always request a kick
        let kick_command = if shutdown_completed {
            KickType::None 
        } else if shutdown_requested {
            KickType::Kick 
        } else { 
            match latched_command {
                KickRequest::KR_DISABLE => {
                    KickType::None
                },
                KickRequest::KR_ARM => {
                    KickType::None
                },
                KickRequest::KR_KICK_NOW => {
                    KickType::Kick
                },
                KickRequest::KR_KICK_TOUCH => {
                    if ball_detected {
                        KickType::Kick
                    } else {
                        KickType::None
                    }
                },
                KickRequest::KR_KICK_CAPTURED => {
                    if ball_detected && rail_voltage > CHARGED_THRESH_VOLTAGE {
                        KickType::Kick
                    } else {
                        KickType::None
                    }
                },
                KickRequest::KR_CHIP_NOW => {
                    KickType::Chip
                },
                KickRequest::KR_CHIP_TOUCH => {
                    if ball_detected {
                        KickType::Chip
                    } else {
                        KickType::None
                    }
                },
                KickRequest::KR_CHIP_CAPTURED => {
                    if ball_detected && rail_voltage > CHARGED_THRESH_VOLTAGE {
                        KickType::Chip
                    } else {
                        KickType::None
                    }
                },
                // possible if packet decoding has corrupted enum val
                _ => {
                    KickType::None
                }
            }
        };

        // the latched command is actionable, and the critical section will be instructed to carry it out in the next segment
        // set clear the command cleared flag, indicating the control board will need to deassert any kick command showing it
        // wants a new unique event
        if kick_command != KickType::None {
            kick_command_cleared = false;
        }

        // perform charge/kick actions. this function handles elec/mechanical safety
        // if telemetry isn't enabled, the control board doesn't want to talk to us, don't permit any actions
        let res = if !telemetry_enabled || error_latched {
            kick_manager.command(battery_voltage, rail_voltage, false, KickType::None, 0.0).await
        } else {
            kick_manager.command(battery_voltage, rail_voltage, charge_hv_rail, kick_command, kick_strength).await
        };

        if res.is_err() {
            error_latched = true;
        }

        // send telemetry packet
        if telemetry_enabled {
            let cur_time = Instant::now();
            if Instant::checked_duration_since(&cur_time, last_packet_sent_time).unwrap().as_millis() > 20 {
                kicker_telemetry_packet._bitfield_1 = KickerTelemetry::new_bitfield_1(shutdown_requested as u32, shutdown_completed as u32, ball_detected as u32, res.is_err() as u32);
                kicker_telemetry_packet.rail_voltage = rail_voltage;
                kicker_telemetry_packet.battery_voltage = battery_voltage;

                // raw interpretaion of a struct for wire transmission is unsafe
                unsafe {
                    // get a slice to packet for transmission
                    let struct_bytes = core::slice::from_raw_parts(
                        (&kicker_telemetry_packet as *const KickerTelemetry) as *const u8,
                        core::mem::size_of::<KickerTelemetry>(),
                    );
        
                    // send the packet
                    let _res = coms_writer.enqueue_copy(struct_bytes);
                }

                last_packet_sent_time = cur_time;
            }
        }

        // LEDs
        if error_latched {
            err_led.set_high();
        } else {
            err_led.set_low();
        }

        if ball_detected {
            ball_detected_led.set_high();
        } else {
            ball_detected_led.set_low();
        }
        // TODO Dotstar

        // loop rate control @1KHz
        ticker.next().await;
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

    let _status_led = Output::new(p.PA11, Level::High, Speed::Low);

    let mut adc = Adc::new(p.ADC, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles71_5);

    // high priority executor handles kicking system
    // High-priority executor: I2C1, priority level 6
    // TODO CHECK THIS IS THE HIGHEST PRIORITY
    unsafe { nvic.set_priority(Interrupt::I2C1, 6 << 4) };
    let spawner = EXECUTOR_HIGH.start(Interrupt::I2C1);
    unwrap!(spawner.spawn(high_pri_kick_task(&COMS_QUEUE_RX, &COMS_QUEUE_TX, adc, p.PB3, p.PB0, p.PB1, p.PA0, p.PA1, p.PA12, p.PA8)));

    //////////////////////////////////
    //  COMMUNICATIONS TASKS SETUP  //
    //////////////////////////////////

    let mut coms_uart_config = Config::default();
    coms_uart_config.baudrate = 1_000_000; // 1 Mbaud
    coms_uart_config.parity = Parity::ParityEven;
    coms_uart_config.stop_bits = StopBits::STOP0P5;

    let coms_usart_int = interrupt::take!(USART1);
    let coms_usart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        coms_usart_int,
        p.DMA1_CH2,
        p.DMA1_CH3,
        coms_uart_config,
    );

    let (coms_uart_tx, coms_uart_rx) = coms_usart.split();

    // low priority executor handles coms and user IO
    // Low priority executor: runs in thread mode, using WFE/SEV
    let lp_executor = EXECUTOR_LOW.init(Executor::new());
    lp_executor.run(|spawner| {
        unwrap!(spawner.spawn(COMS_QUEUE_TX.spawn_task(coms_uart_tx)));
        unwrap!(spawner.spawn(COMS_QUEUE_RX.spawn_task(coms_uart_rx)));
    });
}