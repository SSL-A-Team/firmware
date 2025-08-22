#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]

use core::sync::atomic::Ordering::Relaxed;
use core::sync::atomic::{AtomicBool, AtomicU32};

use ateam_kicker_board::{
    drivers::{breakbeam::Breakbeam, DribblerMotor},
    include_external_cpp_bin,
    pins::{BreakbeamLeftAgpioPin, BreakbeamRightAgpioPin, GreenStatusLedPin},
    tasks::{get_system_config, ClkSource},
    DEBUG_COMS_UART_QUEUES, DEBUG_DRIB_UART_QUEUES,
};

use defmt::*;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{PubSubChannel, Publisher, Subscriber},
};
use {defmt_rtt as _, panic_probe as _};

use libm::{fmaxf, fminf};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    bind_interrupts,
    gpio::{Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt},
    opamp::{OpAmp, OpAmpGain, OpAmpSpeed},
    pac::Interrupt,
    peripherals,
    usart::{self, Config, Parity, StopBits, Uart},
};
use embassy_time::{Duration, Instant, Ticker, Timer};

use ateam_kicker_board::{
    adc_200v_to_rail_voltage, adc_raw_to_v,
    kick_manager::{KickManager, KickType},
    pins::{BlueStatusLedPin, ChargePin, ChipPin, KickPin, PowerRail200vReadPin, RedStatusLedPin},
};

use ateam_lib_stm32::{
    drivers::boot::stm32_interface::{get_bootloader_uart_config, Stm32Interface},
    idle_buffered_uart_spawn_tasks, static_idle_buffered_uart_nl,
    uart::queue::{UartReadQueue, UartWriteQueue},
};

use ateam_kicker_board::image_hash::get_kicker_img_hash;

use ateam_common_packets::bindings::{
    KickRequest::{self, KR_ARM, KR_DISABLE},
    KickerControl, KickerTelemetry, MotorTelemetry,
};

const MAX_KICK_SPEED: f32 = 6.5;
const SHUTDOWN_KICK_SPEED: f32 = 0.20;

pub const CHARGE_TARGET_VOLTAGE: f32 = 182.0;
pub const CHARGE_OVERVOLT_THRESH_VOLTAGE: f32 = 195.0;
pub const CHARGED_THRESH_VOLTAGE: f32 = 170.0;
pub const CHARGE_SAFE_VOLTAGE: f32 = 10.0;

const RAIL_BUFFER_SIZE: usize = 10;

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 32;
const RX_BUF_DEPTH: usize = 3;
static_idle_buffered_uart_nl!(
    COMS,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH,
    DEBUG_COMS_UART_QUEUES
);

include_external_cpp_bin! {DRIB_FW_IMG, "dribbler.bin"}

const DRIB_MAX_TX_PACKET_SIZE: usize = 64;
const DRIB_TX_BUF_DEPTH: usize = 3;
const DRIB_MAX_RX_PACKET_SIZE: usize = 64;
const DRIB_RX_BUF_DEPTH: usize = 20;
static_idle_buffered_uart_nl!(
    DRIB,
    DRIB_MAX_RX_PACKET_SIZE,
    DRIB_RX_BUF_DEPTH,
    DRIB_MAX_TX_PACKET_SIZE,
    DRIB_TX_BUF_DEPTH,
    DEBUG_DRIB_UART_QUEUES
);

static DRIB_VEL_PUBSUB: PubSubChannel<CriticalSectionRawMutex, f32, 1, 1, 1> = PubSubChannel::new();
static DRIB_MULT: AtomicU32 = AtomicU32::new(100);
static BALL_DETECT: AtomicBool = AtomicBool::new(false);
static DRIB_TELEM_PUBSUB: PubSubChannel<CriticalSectionRawMutex, MotorTelemetry, 1, 1, 1> =
    PubSubChannel::new();

#[embassy_executor::task]
async fn high_pri_kick_task(
    coms_reader: &'static UartReadQueue<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, DEBUG_COMS_UART_QUEUES>,
    coms_writer: &'static UartWriteQueue<MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_COMS_UART_QUEUES>,
    mut adc: Adc<'static, embassy_stm32::peripherals::ADC1>,
    charge_pin: ChargePin,
    kick_pin: KickPin,
    chip_pin: ChipPin,
    breakbeam_tx: BreakbeamRightAgpioPin,
    breakbeam_rx: BreakbeamLeftAgpioPin,
    mut rail_pin: PowerRail200vReadPin,
    grn_led_pin: GreenStatusLedPin,
    err_led_pin: RedStatusLedPin,
    ball_detected_led1_pin: BlueStatusLedPin,
    drib_vel_pub: Publisher<'static, CriticalSectionRawMutex, f32, 1, 1, 1>,
    mut drib_telem_sub: Subscriber<'static, CriticalSectionRawMutex, MotorTelemetry, 1, 1, 1>,
) -> ! {
    // pins/safety management
    let charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);
    let mut kick_manager = KickManager::new(charge_pin, kick_pin, chip_pin);

    // image hash
    let kicker_img_hash_kicker = get_kicker_img_hash();

    // debug LEDs
    let mut status_led = Output::new(grn_led_pin, Level::Low, Speed::Low);
    let mut err_led = Output::new(err_led_pin, Level::Low, Speed::Low);
    let mut ball_detected_led1 = Output::new(ball_detected_led1_pin, Level::Low, Speed::Low);

    // TODO dotstars

    let mut breakbeam = Breakbeam::new(breakbeam_tx, breakbeam_rx);

    // coms buffers
    let mut telemetry_enabled: bool; //  = false;
    let mut kicker_control_packet: KickerControl = Default::default();
    let mut kicker_telemetry_packet: KickerTelemetry = Default::default();
    let mut dribbler_motor_telemetry: MotorTelemetry = Default::default();

    // bookkeeping for latched state
    let mut kick_command_cleared: bool = false;
    let mut latched_command = KickRequest::KR_DISABLE;
    let mut error_latched: bool = false;
    let mut charge_hv_rail: bool = false;

    // power down status
    let mut shutdown_requested: bool = false;
    let mut shutdown_completed: bool = false;

    let mut rail_voltage_buffer: [f32; RAIL_BUFFER_SIZE] = [0.0; RAIL_BUFFER_SIZE];
    let mut rail_voltage_filt_indx: usize = 0;

    // loop rate control
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last_packet_sent_time = Instant::now();
    let mut last_packet_received_time = Instant::now();

    breakbeam.enable_tx();

    loop {
        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.blocking_read(&mut vrefint);

        let rail_voltage_cur = adc_200v_to_rail_voltage(adc_raw_to_v(
            adc.blocking_read(&mut rail_pin) as f32,
            vrefint_sample as f32,
        ));

        // Add new battery read to cyclical buffer.
        rail_voltage_buffer[rail_voltage_filt_indx] = rail_voltage_cur;

        // Shift index for next run.
        if rail_voltage_filt_indx == (RAIL_BUFFER_SIZE - 1) {
            rail_voltage_filt_indx = 0;
        } else {
            rail_voltage_filt_indx += 1;
        }

        let rail_voltage_sum: f32 = rail_voltage_buffer.iter().sum();
        // Calculate battery average
        let rail_voltage_ave = rail_voltage_sum / (RAIL_BUFFER_SIZE as f32);

        // let battery_voltage =
        //     adc_v_to_battery_voltage(adc_raw_to_v(adc.read(&mut battery_voltage_pin) as f32, vrefint_sample as f32));
        // optionally pre-flag errors?
        let battery_voltage = 22.5;

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

            drib_vel_pub.publish_immediate(kicker_control_packet.drib_speed);
            DRIB_MULT.store(kicker_control_packet.dribbler_mult(), Relaxed);

            last_packet_received_time = Instant::now();
        }

        // get latest dribbler telem from lower prio task
        while let Some(drib_telem) = drib_telem_sub.try_next_message_pure() {
            dribbler_motor_telemetry = drib_telem;
        }

        let ball_detected = breakbeam.read();
        BALL_DETECT.store(ball_detected, Relaxed);

        // we've missed 20 packet frames from control
        if Instant::now() - last_packet_received_time > Duration::from_millis(200) {
            defmt::warn!("the kicker board is not receiving commands from the control board and dearm itself");
            kicker_control_packet.drib_speed = 0.0;
            kicker_control_packet.kick_speed = 0.0;
            kicker_control_packet.kick_request = KickRequest::KR_DISABLE;
        }

        ///////////////////////////////////////////////
        //  manage repetitive kick commands + state  //
        ///////////////////////////////////////////////

        // update telemetry requests
        telemetry_enabled = kicker_control_packet.telemetry_enabled() != 0;
        if telemetry_enabled {
            status_led.set_high();
        } else {
            status_led.set_low();
        }

        // for now shutdown requests will be latched and a reboot is required to re-power
        if kicker_control_packet.request_power_down() != 0 {
            shutdown_requested = true;
        }

        // check if we've met the criteria for completed shutdown
        if shutdown_requested && rail_voltage_ave < CHARGE_SAFE_VOLTAGE {
            shutdown_completed = true;
        }

        if rail_voltage_ave > CHARGE_OVERVOLT_THRESH_VOLTAGE {
            error_latched = true;
        }

        // charge if were not in shutdown mode AND the control board has requested any state but "DISABLE"
        charge_hv_rail = if shutdown_requested || shutdown_completed {
            false
        } else {
            match kicker_control_packet.kick_request {
                KickRequest::KR_ARM => true,
                KickRequest::KR_DISABLE => false,
                KickRequest::KR_KICK_NOW
                | KickRequest::KR_KICK_CAPTURED
                | KickRequest::KR_KICK_TOUCH
                | KickRequest::KR_CHIP_NOW
                | KickRequest::KR_CHIP_CAPTURED
                | KickRequest::KR_CHIP_TOUCH => charge_hv_rail,
                _ => false,
            }
        };

        // scale kick strength from m/s to duty for the critical section
        // if shutdown is requested and not complete, set kick discharge kick strength to 5%
        let kick_speed = if shutdown_completed {
            0.0
        } else if shutdown_requested {
            SHUTDOWN_KICK_SPEED
        } else {
            fmaxf(0.0, fminf(MAX_KICK_SPEED, kicker_control_packet.kick_speed))
        };

        // if control requests only an ARM or DISABLE, clear the active command
        // software/joystick is not asserting a kick event, so any future kick event is a unique request
        if kicker_control_packet.kick_request == KR_ARM
            || kicker_control_packet.kick_request == KR_DISABLE
        {
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
            if ball_detected {
                // If shutdown requested, person could be picking up the robot.
                // Don't want to kick if ball detected since it might
                // be someone's finger.
                KickType::None
            } else {
                KickType::Kick
            }
        } else {
            match latched_command {
                KickRequest::KR_DISABLE => KickType::None,
                KickRequest::KR_ARM => KickType::None,
                KickRequest::KR_KICK_NOW => KickType::Kick,
                KickRequest::KR_KICK_TOUCH => {
                    if ball_detected {
                        KickType::Kick
                    } else {
                        KickType::None
                    }
                }
                KickRequest::KR_KICK_CAPTURED => {
                    if ball_detected && rail_voltage_ave > CHARGED_THRESH_VOLTAGE {
                        KickType::Kick
                    } else {
                        KickType::None
                    }
                }
                KickRequest::KR_CHIP_NOW => KickType::Chip,
                KickRequest::KR_CHIP_TOUCH => {
                    if ball_detected {
                        KickType::Chip
                    } else {
                        KickType::None
                    }
                }
                KickRequest::KR_CHIP_CAPTURED => {
                    if ball_detected && rail_voltage_ave > CHARGED_THRESH_VOLTAGE {
                        KickType::Chip
                    } else {
                        KickType::None
                    }
                }
                // possible if packet decoding has corrupted enum val
                _ => KickType::None,
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
            kick_manager
                .command(
                    battery_voltage,
                    rail_voltage_ave,
                    false,
                    KickType::None,
                    0.0,
                )
                .await
        } else {
            if kick_command == KickType::Kick || kick_command == KickType::Chip {
                kicker_control_packet.kick_request = match charge_hv_rail {
                    true => KickRequest::KR_ARM,
                    false => KickRequest::KR_DISABLE,
                }
            }

            kick_manager
                .command(
                    battery_voltage,
                    rail_voltage_ave,
                    charge_hv_rail,
                    kick_command,
                    kick_speed,
                )
                .await
        };

        // this will permanently latch an error if the rail voltages are low
        // which we probably don't want on boot up?
        // maybe this error should be clearable and the HV rail OV should not be
        if res.is_err() {
            error_latched = true;
        }

        // send telemetry packet
        if telemetry_enabled {
            let cur_time = Instant::now();
            if Instant::checked_duration_since(&cur_time, last_packet_sent_time)
                .unwrap()
                .as_millis()
                > 20
            {
                kicker_telemetry_packet._bitfield_1 = KickerTelemetry::new_bitfield_1(
                    res.is_err() as u16,
                    dribbler_motor_telemetry.master_error() as u16,
                    shutdown_requested as u16,
                    shutdown_completed as u16,
                    ball_detected as u16,
                    (rail_voltage_ave > CHARGED_THRESH_VOLTAGE) as u16,
                    Default::default(),
                );

                let charge_pct = rail_voltage_ave / CHARGE_TARGET_VOLTAGE;
                kicker_telemetry_packet.charge_pct = (charge_pct * 100.0) as u16;
                kicker_telemetry_packet.rail_voltage = rail_voltage_ave;
                kicker_telemetry_packet.battery_voltage = battery_voltage;

                kicker_telemetry_packet.dribbler_motor = dribbler_motor_telemetry;
                kicker_telemetry_packet
                    .kicker_image_hash
                    .copy_from_slice(&kicker_img_hash_kicker[0..4]);

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
            ball_detected_led1.set_high();
        } else {
            ball_detected_led1.set_low();
        }
        // TODO Dotstar

        // loop rate control @1KHz
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn low_pri_dribble_task(
    drib_motor_interface: Stm32Interface<
        'static,
        DRIB_MAX_RX_PACKET_SIZE,
        DRIB_MAX_TX_PACKET_SIZE,
        DRIB_RX_BUF_DEPTH,
        DRIB_TX_BUF_DEPTH,
        DEBUG_DRIB_UART_QUEUES,
    >,
    mut drib_vel_sub: Subscriber<'static, CriticalSectionRawMutex, f32, 1, 1, 1>,
    drib_telem_pub: Publisher<'static, CriticalSectionRawMutex, MotorTelemetry, 1, 1, 1>,
) -> ! {
    let mut drib_motor = DribblerMotor::new(drib_motor_interface, DRIB_FW_IMG, 1.0);

    defmt::info!("flashing dribbler motor firmware...");
    let force_flash = false;
    let res = drib_motor.init_default_firmware_image(force_flash).await;
    if res.is_err() {
        defmt::error!("failed to load dribbler firmware");
    } else {
        defmt::info!("loaded dribbler firmware");
    }

    drib_motor.leave_reset().await;
    drib_motor.set_telemetry_enabled(true);

    let mut dribbler_ticker = Ticker::every(Duration::from_millis(10));

    let mut lastest_drib_vel = 0.0;
    loop {
        if let Some(drib_vel) = drib_vel_sub.try_next_message_pure() {
            lastest_drib_vel = drib_vel;
            defmt::debug!("got a dribbler velocity update {:?}", lastest_drib_vel);
        }
        drib_motor.process_packets();

        let mut drib_mult = DRIB_MULT.load(Relaxed);
        if drib_mult == 0 || drib_mult > 100 {
            drib_mult = 100;
        }
        let drib_mult_float = (drib_mult as f32) / 100.0f32;

        let mut drib_sp = -1.0 * lastest_drib_vel / 1000.0;
        if !BALL_DETECT.load(Relaxed) {
            drib_sp *= drib_mult_float;
        }
        drib_motor.set_setpoint(drib_sp);

        drib_motor.send_motion_command();

        // TODO log dribbler error upstream
        if drib_motor.read_is_error() {
            defmt::error!("dribbler motor reporting general error");
        }

        if drib_motor.check_hall_error() {
            defmt::error!("dribbler motor reporting hall error");
        }

        drib_telem_pub.publish_immediate(drib_motor.get_latest_state());

        dribbler_ticker.next().await;
    }
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MID: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
#[embassy_stm32::interrupt]
unsafe fn SPI2() {
    EXECUTOR_HIGH.on_interrupt();
}

#[allow(non_snake_case)]
#[embassy_stm32::interrupt]
unsafe fn SPI3() {
    EXECUTOR_MID.on_interrupt();
}

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let stm32_config = get_system_config(ClkSource::InternalOscillator);
    let p = embassy_stm32::init(stm32_config);

    info!("kicker startup!");

    // turn IGBT gate drive supply on
    let _vsw_en = Output::new(p.PE10, Level::High, Speed::Medium);

    // enable opamp for hv measurement
    let mut hv_opamp_inst = OpAmp::new(p.OPAMP3, OpAmpSpeed::HighSpeed);
    let _hv_opamp = hv_opamp_inst.buffer_ext(p.PB0, p.PB1, OpAmpGain::Mul2);

    // config ADC
    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES247_5);

    ///////////////////////
    //  Kick Task Setup  //
    ///////////////////////

    // high priority executor handles kicking system
    // High-priority executor: I2C1, priority level 6
    // TODO CHECK THIS IS THE HIGHEST PRIORITY
    embassy_stm32::interrupt::SPI2.set_priority(embassy_stm32::interrupt::Priority::P6);
    let hp_spawner = EXECUTOR_HIGH.start(Interrupt::SPI2);

    embassy_stm32::interrupt::SPI3.set_priority(embassy_stm32::interrupt::Priority::P7);
    let mp_spawner = EXECUTOR_MID.start(Interrupt::SPI3);

    // spawn the task at the highest prio
    unwrap!(hp_spawner.spawn(high_pri_kick_task(
        COMS_IDLE_BUFFERED_UART.get_uart_read_queue(),
        COMS_IDLE_BUFFERED_UART.get_uart_write_queue(),
        adc,
        p.PB15,
        p.PD9,
        p.PD8,
        p.PC2,
        p.PC0,
        p.PC3,
        p.PB9,
        p.PE0,
        p.PE1,
        DRIB_VEL_PUBSUB
            .publisher()
            .expect("failed to get dribbler vel publisher for kick task"),
        DRIB_TELEM_PUBSUB
            .subscriber()
            .expect("failed to get drib telem sub for kick task")
    )));

    //////////////////////////////////
    //  COMMUNICATIONS TASKS SETUP  //
    //////////////////////////////////

    let mut coms_uart_config = Config::default();
    coms_uart_config.baudrate = 2_000_000; // 2 Mbaud
    coms_uart_config.parity = Parity::ParityEven;
    coms_uart_config.stop_bits = StopBits::STOP1;

    let coms_usart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        p.DMA2_CH7,
        p.DMA2_CH2,
        coms_uart_config,
    )
    .unwrap();

    COMS_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(mp_spawner, COMS, coms_usart);

    ////////////////////////////
    //  Dribbler Motor Setup  //
    ////////////////////////////

    let initial_motor_controller_uart_config = get_bootloader_uart_config();
    let drib_uart = Uart::new(
        p.USART3,
        p.PE15,
        p.PB10,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        initial_motor_controller_uart_config,
    )
    .unwrap();

    DRIB_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(mp_spawner, DRIB, drib_uart);

    let drib_motor_interface = Stm32Interface::new_from_pins(
        &DRIB_IDLE_BUFFERED_UART,
        DRIB_IDLE_BUFFERED_UART.get_uart_read_queue(),
        DRIB_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PE13,
        p.PE14,
        Pull::None,
        true,
    );

    spawner
        .spawn(low_pri_dribble_task(
            drib_motor_interface,
            DRIB_VEL_PUBSUB
                .subscriber()
                .expect("failed to get dribler vel subscriber for drib task"),
            DRIB_TELEM_PUBSUB
                .publisher()
                .expect("failed to get drib telem pub for dribble task"),
        ))
        .expect("failed to spawn dribble task");

    /////////////////////////////////////////
    //  spin and allow other tasks to run  //
    /////////////////////////////////////////

    loop {
        Timer::after_millis(1000).await;
    }
}
