#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(sync_unsafe_cell)]

//! Lever arm torque profiling bench.
//!
//! Companion to profile-wheel-torque, which compares the firmware's current
//! estimates against each other with no physical fixture. This bench instead
//! validates against a physically measured reaction force. A lever arm is
//! clamped to the motor shaft and rests on a kitchen scale; the rotor is
//! therefore stalled. The
//! board commands a current setpoint, the motor develops torque against the
//! lever, and the scale reads the reaction force. The operator reads grams off
//! the scale and compares against what each of the firmware's four concurrent
//! current estimates predicts.
//!
//! Set LEVER_ARM_MM below to the distance from the shaft centreline to the
//! contact point on the scale before flashing.
//!
//! Controls:
//!   LEFT / RIGHT  select active wheel (counter-clockwise / clockwise order)
//!   UP / DOWN     current setpoint +/- CURRENT_STEP_MA, auto-repeats when held.
//!                 The setpoint is signed, so holding DOWN through zero reverses
//!                 torque direction.
//!   CENTER        arm / disarm the output
//!
//! Nothing is driven until the output is armed. While armed a full data row is
//! logged at ROW_LOG_HZ and streamed over USB CDC as a TorqueSample.
//!
//! Stall is a no-cooling condition, so the output auto-disarms after
//! ARMED_TIMEOUT_S, on any motor error, and if the wheel is found to be turning
//! (which means the lever slipped and every torque reading is invalid).
//!
//! Two things to keep in mind reading the numbers at stall:
//!   - Duty is tiny (order 1%), which is exactly where the Phase 0 duty
//!     correction runs out of resolution. Watch the corr_valid flag and D_arr;
//!     below D_arr 10 the corrected value is not a measurement.
//!   - Back-EMF is zero, so the Phase 3 model degenerates to D * Vbus / R_loop
//!     and the velocity source selection has no effect here. It only starts to
//!     matter once the shaft is turning.

use ateam_common_packets::bindings::CcmMotionControlType;
use ateam_lib_stm32::{
    drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart,
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Input, Pull},
    interrupt,
    pac::Interrupt,
    peripherals,
    usart::Uart,
};
use embassy_stm32::{peripherals::USB_OTG_HS, usb::Driver};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{PubSubChannel, Subscriber},
};

use defmt_rtt as _;

use ateam_control_board::{
    get_system_config, include_external_cpp_bin, motor::CurrentControlledMotor, SystemIrqs,
};

use embassy_time::{Duration, Instant, Ticker, Timer};
use panic_probe as _;

use embassy_usb::driver::EndpointError;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    UsbDevice,
};

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

include_external_cpp_bin! {CURRENT_CONTROLLED_WHEEL_IMAGE, "wheel-torque.bin"}

const MAX_TX_PACKET_SIZE: usize = 80;
const TX_BUF_DEPTH: usize = 5;
// Sized from the packet definition so it tracks CcmResponse, which grew to
// 84 bytes with the current sense estimator telemetry.
const MAX_RX_PACKET_SIZE: usize =
    core::mem::size_of::<ateam_common_packets::bindings::CcmResponse>();
const RX_BUF_DEPTH: usize = 5;

static_idle_buffered_uart!(FRONT_LEFT,  MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(FRONT_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_RIGHT,  MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_LEFT,   MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);

static mut EP_OUT_BUFFER_CELL: [u8; 4096] = [0; 4096];
static mut CONFIG_DESCRIPTOR_CELL: [u8; 256] = [0; 256];
static mut BOS_DESCRIPTOR_CELL: [u8; 256] = [0; 256];
static mut CONTROL_BUF_CELL: [u8; 4096] = [0; 4096];
static mut USB_STATE_CELL: State = State::new();

bind_interrupts!(struct Irqs {
    OTG_HS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_HS>;
});

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

////////////////////
//  bench config  //
////////////////////

/// Distance from the shaft centreline to where the lever arm contacts the
/// scale. This is the one number that has to match the physical fixture.
const LEVER_ARM_MM: f32 = 20.0;

/// Nanotec DF45M024053-A2 torque constant, N*m/A. Note the manufacturer's
/// characterization does not extend below 500 mA, so anything this predicts
/// under that is an extrapolation.
const MOTOR_KT_NM_PER_A: f32 = 0.0335;

const GRAVITY_M_PER_S2: f32 = 9.80665;

/// Setpoint granularity, and the ceiling. The motor firmware independently
/// clamps stall current to MAX_CURR_WHEEL_NOT_TURNING (2160 mA), so this stays
/// below that to keep the commanded value and the applied value identical.
const CURRENT_STEP_MA: i16 = 50;
const MAX_SETPOINT_MA: i16 = 2000;

/// Stall means no rotor cooling, so armed time is bounded.
const ARMED_TIMEOUT_S: u64 = 20;

/// A stalled shaft should read essentially zero. Anything above this while armed
/// means the lever slipped off the scale and the torque reading is meaningless.
const STALL_VIOLATION_RADS: f32 = 20.0;

/// Data row logging rate while armed.
const ROW_LOG_HZ: u32 = 2;

// Standard coordinate convention: FL=0, BL=1, BR=2, FR=3
// Clockwise physical order: FL(0) -> FR(3) -> BR(2) -> BL(1) -> FL(0)
const WHEEL_NAMES: [&str; 4] = ["FL", "BL", "BR", "FR"];
const CW_NEXT: [usize; 4] = [3, 0, 1, 2]; // FL->FR, BL->FL, BR->BL, FR->BR
const CCW_NEXT: [usize; 4] = [1, 2, 3, 0]; // FL->BL, BL->BR, BR->FR, FR->FL

// 500 us control tick
const TICK_US: u64 = 500;
const TICKS_PER_S: u32 = (1_000_000 / TICK_US) as u32;
/// Per-button debounce cooldown, in ticks (100 ms).
const BTN_COOLDOWN: u32 = 200;
/// How long a button must be held before it starts auto-repeating (300 ms),
/// and how fast it repeats after that (50 ms, so 20 steps/s).
const BTN_REPEAT_DELAY: u32 = 600;
const BTN_REPEAT_PERIOD: u32 = 100;

type TorqueTestMotor = CurrentControlledMotor<
    'static,
    MAX_RX_PACKET_SIZE,
    MAX_TX_PACKET_SIZE,
    RX_BUF_DEPTH,
    TX_BUF_DEPTH,
    false,
>;

/// One bench sample, streamed over USB CDC. Deliberately carries the raw
/// firmware values rather than derived torque so the host can recompute with a
/// different Kt or lever length without reflashing.
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct TorqueSample {
    timestamp_ms: u32,
    wheel_index: u8,
    armed: u8,
    cs_flags: u8,
    _pad: u8,
    setpoint_ma: i16,
    current_filt_ma: u16,
    current_unfilt_ma: u16,
    current_duty_corrected_ma: u16,
    current_model_ma: i16,
    duty_arr: u16,
    vel_est_used_drads: i16,
    wheel_vel_drads: i16,
    bus_voltage_mv: u16,
    motor_voltage_cmd_mv: u16,
}

static SAMPLE_PUBSUB: PubSubChannel<CriticalSectionRawMutex, TorqueSample, 3, 1, 1> =
    PubSubChannel::new();

/// Torque a current would produce, in mN*m.
fn current_ma_to_torque_mnm(current_ma: f32) -> f32 {
    (current_ma / 1000.0) * MOTOR_KT_NM_PER_A * 1000.0
}

/// What the scale should read for a given current, in grams.
///
/// F = tau / r, and the scale reports mass, so divide out g.
fn current_ma_to_scale_grams(current_ma: f32) -> f32 {
    let torque_nm = (current_ma / 1000.0) * MOTOR_KT_NM_PER_A;
    let force_n = torque_nm / (LEVER_ARM_MM / 1000.0);
    (force_n / GRAVITY_M_PER_S2) * 1000.0
}

/// Estimate error against the commanded setpoint, in percent. Returns 0 at a
/// zero setpoint rather than an infinity, since a zero setpoint has no defined
/// relative error.
fn err_pct(estimate_ma: f32, setpoint_ma: f32) -> f32 {
    if setpoint_ma.abs() < 1.0 {
        return 0.0;
    }
    (estimate_ma - setpoint_ma.abs()) / setpoint_ma.abs() * 100.0
}

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("embassy HAL configured.");

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P7,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    // Buttons: active low with internal pull-up
    let btn_enter = Input::new(p.PE11, Pull::Up); // center
    let btn_left = Input::new(p.PE12, Pull::Up);
    let btn_right = Input::new(p.PE13, Pull::Up);
    let btn_up = Input::new(p.PE14, Pull::Up);
    let btn_down = Input::new(p.PE15, Pull::Up);

    let initial_uart_config = stm32_interface::get_bootloader_uart_config();

    // Uart::new(uart, rx_pin, tx_pin, tx_dma, rx_dma, irqs, config)
    let fl_uart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        p.DMA1_CH0,
        p.DMA1_CH1,
        SystemIrqs,
        initial_uart_config,
    )
    .unwrap();
    let fr_uart = Uart::new(
        p.USART3,
        p.PD9,
        p.PD8,
        p.DMA1_CH6,
        p.DMA1_CH7,
        SystemIrqs,
        initial_uart_config,
    )
    .unwrap();
    let br_uart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        p.DMA1_CH4,
        p.DMA1_CH5,
        SystemIrqs,
        initial_uart_config,
    )
    .unwrap();
    let bl_uart = Uart::new(
        p.USART10,
        p.PE2,
        p.PE3,
        p.DMA1_CH2,
        p.DMA1_CH3,
        SystemIrqs,
        initial_uart_config,
    )
    .unwrap();

    FRONT_LEFT_IDLE_BUFFERED_UART.init();
    FRONT_RIGHT_IDLE_BUFFERED_UART.init();
    BACK_RIGHT_IDLE_BUFFERED_UART.init();
    BACK_LEFT_IDLE_BUFFERED_UART.init();

    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, FRONT_LEFT, fl_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, FRONT_RIGHT, fr_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, BACK_RIGHT, br_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, BACK_LEFT, bl_uart);

    // Motors indexed by coordinate convention: FL=0, BL=1, BR=2, FR=3
    let mut motors: [TorqueTestMotor; 4] = [
        TorqueTestMotor::new_from_pins(
            &FRONT_LEFT_IDLE_BUFFERED_UART,
            FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PF5.into(),
            p.PF4.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
        TorqueTestMotor::new_from_pins(
            &BACK_LEFT_IDLE_BUFFERED_UART,
            BACK_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            BACK_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PE5.into(),
            p.PE4.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
        TorqueTestMotor::new_from_pins(
            &BACK_RIGHT_IDLE_BUFFERED_UART,
            BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PG7.into(),
            p.PG8.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
        TorqueTestMotor::new_from_pins(
            &FRONT_RIGHT_IDLE_BUFFERED_UART,
            FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PB12.into(),
            p.PB13.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
    ];

    let sample_pub = SAMPLE_PUBSUB
        .publisher()
        .expect("could not get sample publisher");
    let usb_subscriber = SAMPLE_PUBSUB
        .subscriber()
        .expect("could not get sample subscriber");

    defmt::info!("Setting up USB...");
    let mut usb_hw_config = embassy_stm32::usb::Config::default();
    usb_hw_config.vbus_detection = false;

    let ep_out_buffer: &'static mut [u8; 4096] = unsafe { &mut (*(&raw mut EP_OUT_BUFFER_CELL)) };
    let usb_driver = embassy_stm32::usb::Driver::new_fs(
        p.USB_OTG_HS,
        Irqs,
        p.PA12,
        p.PA11,
        ep_out_buffer,
        usb_hw_config,
    );

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("A-Team");
    usb_config.product = Some("Control Board");
    usb_config.serial_number = Some("12345678");

    let usb_state: &'static mut State = unsafe { &mut (*(&raw mut USB_STATE_CELL)) };
    let config_descriptor: &'static mut [u8; 256] =
        unsafe { &mut (*(&raw mut CONFIG_DESCRIPTOR_CELL)) };
    let bos_descriptor: &'static mut [u8; 256] = unsafe { &mut (*(&raw mut BOS_DESCRIPTOR_CELL)) };
    let control_buf: &'static mut [u8; 4096] = unsafe { &mut (*(&raw mut CONTROL_BUF_CELL)) };

    let mut usb_builder = embassy_usb::Builder::new(
        usb_driver,
        usb_config,
        config_descriptor,
        bos_descriptor,
        &mut [],
        control_buf,
    );
    let cdc_usb_class = CdcAcmClass::new(&mut usb_builder, usb_state, 64);
    let usb_device_driver = usb_builder.build();

    main_spawner
        .spawn(usb_ll_driver_task(usb_device_driver).expect("failed to spawn USB driver task"));
    main_spawner
        .spawn(usb_writer_task(cdc_usb_class, usb_subscriber).expect("failed to spawn USB task"));

    defmt::info!("Flashing motors...");
    for i in 0..4usize {
        let res = motors[i].init_default_firmware_image(true).await;
        if res.is_ok() {
            defmt::info!("motor {} ({}) flashed.", i, WHEEL_NAMES[i]);
        } else {
            defmt::error!("motor {} ({}) failed to flash!", i, WHEEL_NAMES[i]);
        }
    }

    for motor in motors.iter_mut() {
        motor.set_motion_type(CcmMotionControlType::CCM_MCT_CURRENT);
        motor.set_current_setpoint(0);
        motor.set_telemetry_enabled(true);
        motor.set_motion_enabled(true);
    }

    for i in 0..4usize {
        motors[i].reset().await;
    }
    Timer::after_millis(100).await;

    // Bench state
    let mut active_wheel: usize = 0;
    let mut setpoint_ma: i16 = 0;
    let mut armed = false;
    let mut armed_at = Instant::now();
    let mut last_seq_num: u8 = 0;

    let row_log_divisor: u32 = TICKS_PER_S / ROW_LOG_HZ;
    let mut row_log_ctr: u32 = 0;
    let mut ctr: usize = 0;

    // Button edge-detection and debounce state
    let mut prev_enter = false;
    let mut prev_left = false;
    let mut prev_right = false;
    let mut cd_enter: u32 = 0;
    let mut cd_left: u32 = 0;
    let mut cd_right: u32 = 0;

    // UP/DOWN use hold-to-repeat rather than plain edge detection
    let mut held_up: u32 = 0;
    let mut held_down: u32 = 0;

    defmt::info!(
        "Torque bench ready. Wheel {}, lever arm {}mm, Kt {}Nm/A.",
        WHEEL_NAMES[active_wheel],
        LEVER_ARM_MM,
        MOTOR_KT_NM_PER_A
    );
    defmt::info!(
        "LEFT/RIGHT select wheel, UP/DOWN adjust current by {}mA (hold to repeat), CENTER arms output.",
        CURRENT_STEP_MA
    );

    let mut ticker = Ticker::every(Duration::from_micros(TICK_US));
    loop {
        for motor in motors.iter_mut() {
            motor.process_packets();
        }

        //////////////////
        //  safety cuts //
        //////////////////

        if armed {
            if motors[active_wheel].read_is_error() {
                armed = false;
                setpoint_ma = 0;
                defmt::error!(
                    "{} reported an error. Output disarmed.",
                    WHEEL_NAMES[active_wheel]
                );
            } else if (Instant::now() - armed_at).as_secs() >= ARMED_TIMEOUT_S {
                armed = false;
                setpoint_ma = 0;
                defmt::warn!(
                    "Armed for {}s with a stalled rotor. Output disarmed, let the motor cool.",
                    ARMED_TIMEOUT_S
                );
            } else if libm::fabsf(motors[active_wheel].read_rads()) > STALL_VIOLATION_RADS {
                armed = false;
                setpoint_ma = 0;
                defmt::error!(
                    "{} is turning at {}rad/s - the rotor is not stalled, so the lever has slipped. Output disarmed; discard the last reading.",
                    WHEEL_NAMES[active_wheel],
                    motors[active_wheel].read_rads()
                );
            }
        }

        //////////////
        //  inputs  //
        //////////////

        if cd_enter > 0 {
            cd_enter -= 1;
        }
        if cd_left > 0 {
            cd_left -= 1;
        }
        if cd_right > 0 {
            cd_right -= 1;
        }

        let now_enter = btn_enter.is_low();
        let now_left = btn_left.is_low();
        let now_right = btn_right.is_low();
        let now_up = btn_up.is_low();
        let now_down = btn_down.is_low();

        // CENTER: arm / disarm
        if now_enter && !prev_enter && cd_enter == 0 {
            cd_enter = BTN_COOLDOWN;
            armed = !armed;
            if armed {
                armed_at = Instant::now();
                defmt::info!(
                    "ARMED. {} at {}mA. Expected scale reading {}g.",
                    WHEEL_NAMES[active_wheel],
                    setpoint_ma,
                    current_ma_to_scale_grams(setpoint_ma as f32)
                );
            } else {
                defmt::info!("DISARMED.");
            }
        }

        // RIGHT / LEFT: select active wheel. Always disarms - the lever has to be
        // physically moved to the new wheel anyway.
        if now_right && !prev_right && cd_right == 0 {
            cd_right = BTN_COOLDOWN;
            active_wheel = CW_NEXT[active_wheel];
            armed = false;
            setpoint_ma = 0;
            last_seq_num = 0;
            defmt::info!(
                "Active wheel: {} (clockwise). Output disarmed.",
                WHEEL_NAMES[active_wheel]
            );
        }

        if now_left && !prev_left && cd_left == 0 {
            cd_left = BTN_COOLDOWN;
            active_wheel = CCW_NEXT[active_wheel];
            armed = false;
            setpoint_ma = 0;
            last_seq_num = 0;
            defmt::info!(
                "Active wheel: {} (counter-clockwise). Output disarmed.",
                WHEEL_NAMES[active_wheel]
            );
        }

        // UP / DOWN: adjust the signed setpoint, with hold-to-repeat so a sweep
        // does not need a few dozen individual presses.
        let step_up = button_step(now_up, &mut held_up);
        let step_down = button_step(now_down, &mut held_down);

        if step_up || step_down {
            let prev = setpoint_ma;
            if step_up {
                setpoint_ma = setpoint_ma.saturating_add(CURRENT_STEP_MA);
            } else {
                setpoint_ma = setpoint_ma.saturating_sub(CURRENT_STEP_MA);
            }
            setpoint_ma = setpoint_ma.clamp(-MAX_SETPOINT_MA, MAX_SETPOINT_MA);

            if setpoint_ma != prev {
                defmt::info!(
                    "Setpoint {}mA -> predicted torque {}mNm, scale {}g{}",
                    setpoint_ma,
                    current_ma_to_torque_mnm(setpoint_ma as f32),
                    current_ma_to_scale_grams(setpoint_ma as f32),
                    if armed { "" } else { " (output disarmed)" }
                );
            }
        }

        prev_enter = now_enter;
        prev_left = now_left;
        prev_right = now_right;

        ////////////////////////
        //  sample and report //
        ////////////////////////

        let cur_seq = motors[active_wheel].get_latest_state_seqnum();
        if cur_seq != last_seq_num {
            last_seq_num = cur_seq;

            let m = &motors[active_wheel];
            let sample = TorqueSample {
                timestamp_ms: Instant::now().as_millis() as u32,
                wheel_index: active_wheel as u8,
                armed: armed as u8,
                cs_flags: m.read_cs_flags(),
                _pad: 0,
                setpoint_ma,
                current_filt_ma: m.read_current_filt_ma(),
                current_unfilt_ma: m.read_current_unfilt_ma(),
                current_duty_corrected_ma: m.read_current_duty_corrected_ma(),
                current_model_ma: m.read_current_model_ma(),
                duty_arr: m.read_cs_duty_arr(),
                vel_est_used_drads: m.read_cs_vel_est_used_drads(),
                wheel_vel_drads: (m.read_rads() * 10.0) as i16,
                bus_voltage_mv: (m.read_vbus_voltage() * 1000.0) as u16,
                motor_voltage_cmd_mv: m.read_vmotor_voltage_mv(),
            };
            sample_pub.publish_immediate(sample);

            row_log_ctr += 1;
            if row_log_ctr >= row_log_divisor {
                row_log_ctr = 0;
                log_row(&motors[active_wheel], active_wheel, setpoint_ma, armed);
            }
        }

        // Apply setpoints: the active motor gets the setpoint only while armed,
        // everything else is held at zero.
        for (i, motor) in motors.iter_mut().enumerate() {
            let sp = if i == active_wheel && armed {
                setpoint_ma
            } else {
                0i16
            };
            motor.set_current_setpoint(sp);
        }

        if ctr % 2 == 0 {
            for motor in motors.iter_mut() {
                motor.send_motion_command();
            }
        }

        ctr += 1;
        ticker.next().await;
    }
}

/// Edge-plus-hold-repeat for a single button. Returns true on the tick the
/// button should fire, updating the caller's held-tick counter.
fn button_step(pressed: bool, held_ticks: &mut u32) -> bool {
    if !pressed {
        *held_ticks = 0;
        return false;
    }

    let prev = *held_ticks;
    *held_ticks = prev + 1;

    if prev == 0 {
        // initial press
        return true;
    }
    if prev < BTN_REPEAT_DELAY {
        return false;
    }
    (prev - BTN_REPEAT_DELAY) % BTN_REPEAT_PERIOD == 0
}

/// Full comparison row: every current estimate, what torque and scale reading it
/// implies, and how far each one sits from the commanded setpoint.
fn log_row(m: &TorqueTestMotor, wheel: usize, setpoint_ma: i16, armed: bool) {
    let sp = setpoint_ma as f32;
    let sp_abs = libm::fabsf(sp);

    let filt = m.read_current_filt_ma() as f32;
    let unfilt = m.read_current_unfilt_ma() as f32;
    let duty_corr = m.read_current_duty_corrected_ma() as f32;
    let model = m.read_current_model_ma() as f32;

    let duty = m.read_cs_duty();

    defmt::info!(
        "== {} {} sp {}mA  duty {}% (D_arr {}, corr_valid {})  vbus {}V  vel {}rad/s  sync {}",
        WHEEL_NAMES[wheel],
        if armed { "ARMED" } else { "idle" },
        setpoint_ma,
        duty * 100.0,
        m.read_cs_duty_arr(),
        m.read_cs_duty_correction_valid(),
        m.read_vbus_voltage(),
        m.read_rads(),
        m.read_cs_sync_sampling_enabled()
    );

    defmt::info!(
        "   current mA   filt {} | unfilt {} | duty_corr {} | model {} (valid {})",
        filt,
        unfilt,
        duty_corr,
        model,
        m.read_cs_model_valid()
    );

    defmt::info!(
        "   err vs sp %  filt {} | unfilt {} | duty_corr {} | model {}",
        err_pct(filt, sp),
        err_pct(unfilt, sp),
        err_pct(duty_corr, sp),
        err_pct(model, sp)
    );

    // The investigation predicts filt/setpoint ~= duty, because the filtered tap
    // reads D * I_phase. If that ratio tracks duty, the duty weighting is the
    // whole story.
    if sp_abs >= 1.0 {
        defmt::info!(
            "   filt/sp {} vs duty {}  (equal => filtered tap is reading bus current)",
            filt / sp_abs,
            duty
        );
    }

    defmt::info!(
        "   scale g      cmd {} | filt {} | unfilt {} | duty_corr {} | model {}",
        current_ma_to_scale_grams(sp),
        current_ma_to_scale_grams(filt),
        current_ma_to_scale_grams(unfilt),
        current_ma_to_scale_grams(duty_corr),
        current_ma_to_scale_grams(model)
    );

    defmt::info!(
        "   torque mNm   cmd {} | filt {} | unfilt {} | duty_corr {} | model {}",
        current_ma_to_torque_mnm(sp),
        current_ma_to_torque_mnm(filt),
        current_ma_to_torque_mnm(unfilt),
        current_ma_to_torque_mnm(duty_corr),
        current_ma_to_torque_mnm(model)
    );
}

#[embassy_executor::task]
async fn usb_ll_driver_task(mut usb_device: UsbDevice<'static, Driver<'static, USB_OTG_HS>>) {
    loop {
        defmt::info!("starting USB ll driver core.");
        usb_device.run().await;
        defmt::panic!("usb device driver task returned!");
    }
}

#[embassy_executor::task]
async fn usb_writer_task(
    mut usb_class: CdcAcmClass<'static, Driver<'static, USB_OTG_HS>>,
    mut packet_sub: Subscriber<'static, CriticalSectionRawMutex, TorqueSample, 3, 1, 1>,
) {
    loop {
        defmt::info!("USB task - waiting connection...");
        usb_class.wait_connection().await;
        defmt::info!("Connected");

        loop {
            let sample = packet_sub.next_message_pure().await;

            let struct_bytes = unsafe {
                core::slice::from_raw_parts(
                    (&sample as *const TorqueSample) as *const u8,
                    core::mem::size_of::<TorqueSample>(),
                )
            };

            let res = usb_class.write_packet(struct_bytes).await;
            if res.is_err() {
                match res.err().unwrap() {
                    EndpointError::BufferOverflow => {
                        defmt::error!("USB transmit buffer overflowed");
                    }
                    EndpointError::Disabled => {
                        defmt::warn!("USB disconnected.");
                        break;
                    }
                }
            }
        }
    }
}
