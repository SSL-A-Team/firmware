#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(sync_unsafe_cell)]

//! Bus power vs. current-sense estimate comparison bench.
//!
//! Companion to profile-wheel-torque-level, which validates torque against a
//! physical lever-and-scale fixture. This bench needs no fixture at all - it
//! runs the wheel at a commanded current setpoint (stalled or free-spinning,
//! either is fine) and compares three concurrent readings of the same current
//! against each other and against the setpoint:
//!
//!   - bus power:      V_bus * I_filt, where I_filt is the filtered current
//!                      sense tap. That tap's time average is bus current,
//!                      `D * I_phase`, not phase current, so treating it as
//!                      torque-producing current is the "naive"/uncorrected
//!                      read - it undershoots by roughly a factor of duty.
//!   - bus power corrected to torque: the duty-corrected estimate, which
//!                      divides the duty weighting back out of the bus current
//!                      to recover phase current. Check corr_valid; at very
//!                      low duty (below D_arr ~10) the correction has no
//!                      resolution and passes the uncorrected value through.
//!   - sync sampling:   the pre-filter tap sampled inside the PWM on-window.
//!                      Real phase current only when the firmware was built
//!                      with synchronous shunt sampling (see the `sync` flag
//!                      in each row); otherwise this channel is noise.
//!
//! All three are converted to torque and to an equivalent tangential force at
//! the wheel's rolling radius, assuming WHEEL_DIAMETER_MM below. When sync
//! sampling is enabled, its reading is the closest thing to ground truth
//! here, so each row also reports how far the duty-corrected estimate sits
//! from it.
//!
//! Two control modes, toggled from the board:
//!
//!   CURRENT  closes the STSPIN's current PI loop on the setpoint.
//!   VOLTAGE  commands duty open loop, taking the PI out of the path entirely.
//!            Duty is then steady period to period, which is the condition the
//!            synchronous sample assumes - one sample per PWM period is only a
//!            valid estimate of average phase current when duty is not moving.
//!            Use this to tell a current loop limit cycle apart from a
//!            commutation problem: if the whine and the uneven torque survive a
//!            steady duty, the PI is not what is producing them.
//!
//! At stall, back-EMF is zero, so a voltage command maps to current through
//! I = V / R_LOOP_OHMS. The predicted stall current is logged with each step.
//!
//! Controls:
//!   LEFT / RIGHT  select active wheel (counter-clockwise / clockwise order)
//!   UP / DOWN     setpoint +/- one step, auto-repeats when held. The setpoint
//!                 is signed, so holding DOWN through zero reverses torque
//!                 direction.
//!   CENTER        short press arms / disarms; hold for BTN_LONG_PRESS switches
//!                 control mode (and disarms)
//!
//! Nothing is driven until the output is armed. While armed a full data row is
//! logged at ROW_LOG_HZ and streamed over USB CDC as a TorqueSample.
//!
//! The output auto-disarms after ARMED_TIMEOUT_S or on any motor error.

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

/// Wheel diameter, for converting torque to an equivalent tangential force at
/// the wheel's rolling radius.
const WHEEL_DIAMETER_MM: f32 = 60.0;
const WHEEL_RADIUS_MM: f32 = WHEEL_DIAMETER_MM / 2.0;

/// Nanotec DF45M024053-A2 torque constant, N*m/A. Note the manufacturer's
/// characterization does not extend below 500 mA, so anything this predicts
/// under that is an extrapolation.
const MOTOR_KT_NM_PER_A: f32 = 0.0335;

/// Setpoint granularity, and the ceiling. The motor firmware independently
/// clamps stall current to MAX_CURR_WHEEL_NOT_TURNING (2160 mA), so this stays
/// below that to keep the commanded value and the applied value identical.
const CURRENT_STEP_MA: i16 = 50;
const MAX_SETPOINT_MA: i16 = 2000;

/// Voltage mode granularity and ceiling. One ARR count of duty is ~41.7 mV at a
/// 25 V bus, so a 50 mV step is roughly the hardware resolution - anything finer
/// just aliases onto the same duty. The ceiling is the voltage that would draw
/// MAX_SETPOINT_MA at stall.
const VOLTAGE_STEP_MV: i16 = 50;
const MAX_SETPOINT_MV: i16 = 1800;

/// Total series resistance during the active vector: 0.8 ohm of winding, two
/// STL8N10F7 at 17 mohm, and the 50 mohm shunt. At stall there is no back-EMF,
/// so a voltage command lands at I = V / R_loop. Dividing mV by ohms gives mA
/// directly.
const R_LOOP_OHMS: f32 = 0.88;

/// Bound on armed time regardless of whether the shaft is stalled or turning.
const ARMED_TIMEOUT_S: u64 = 60;

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
/// How long CENTER has to be held to mean "switch mode" rather than "arm" (1s).
/// The switch fires on crossing the threshold rather than on release, so the
/// operator gets the log line without having to guess how long to hold.
const BTN_LONG_PRESS: u32 = 2000;

type TorqueTestMotor = CurrentControlledMotor<
    'static,
    MAX_RX_PACKET_SIZE,
    MAX_TX_PACKET_SIZE,
    RX_BUF_DEPTH,
    TX_BUF_DEPTH,
    false,
>;

/// Which firmware control path the bench drives. Cast to u8 into TorqueSample,
/// so the discriminants are pinned rather than left to the compiler.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum ControlMode {
    Current = 0,
    Voltage = 1,
}

impl ControlMode {
    fn name(self) -> &'static str {
        match self {
            ControlMode::Current => "CURRENT",
            ControlMode::Voltage => "VOLTAGE",
        }
    }

    fn toggled(self) -> Self {
        match self {
            ControlMode::Current => ControlMode::Voltage,
            ControlMode::Voltage => ControlMode::Current,
        }
    }
}

/// One bench sample, streamed over USB CDC. Deliberately carries the raw
/// firmware values rather than derived torque so the host can recompute with a
/// different Kt or wheel radius without reflashing.
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct TorqueSample {
    timestamp_ms: u32,
    wheel_index: u8,
    armed: u8,
    cs_flags: u8,
    /// 0 = current mode, 1 = voltage open loop.
    mode: u8,
    setpoint_ma: i16,
    setpoint_mv: i16,
    bus_current_ma: u16,
    corrected_current_ma: u16,
    sync_current_ma: u16,
    duty_arr: u16,
    wheel_vel_drads: i16,
    bus_voltage_mv: u16,
}

static SAMPLE_PUBSUB: PubSubChannel<CriticalSectionRawMutex, TorqueSample, 3, 1, 1> =
    PubSubChannel::new();

/// Torque a current would produce, in mN*m.
fn current_ma_to_torque_mnm(current_ma: f32) -> f32 {
    (current_ma / 1000.0) * MOTOR_KT_NM_PER_A * 1000.0
}

/// Equivalent tangential force at the wheel's rolling radius for a given
/// current, in Newtons.
fn current_ma_to_wheel_force_n(current_ma: f32) -> f32 {
    let torque_nm = (current_ma / 1000.0) * MOTOR_KT_NM_PER_A;
    torque_nm / (WHEEL_RADIUS_MM / 1000.0)
}

/// Current a voltage command lands at with the rotor stalled, in mA. Only valid
/// at stall - once the shaft turns, back-EMF subtracts and this reads high.
fn voltage_mv_to_stall_current_ma(voltage_mv: f32) -> f32 {
    voltage_mv / R_LOOP_OHMS
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

/// a relative to b, in percent. Returns 0 when b is ~zero rather than an
/// infinity, since that ratio carries no information at zero current.
fn pct_diff(a_ma: f32, b_ma: f32) -> f32 {
    if b_ma.abs() < 1.0 {
        return 0.0;
    }
    (a_ma - b_ma) / b_ma.abs() * 100.0
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
    let mut mode = ControlMode::Current;
    // Kept separately so switching modes does not carry a current setpoint over
    // into a voltage command, where the number would mean something else.
    let mut setpoint_ma: i16 = 0;
    let mut setpoint_mv: i16 = 0;
    let mut armed = false;
    let mut armed_at = Instant::now();
    let mut last_seq_num: u8 = 0;

    let row_log_divisor: u32 = TICKS_PER_S / ROW_LOG_HZ;
    let mut row_log_ctr: u32 = 0;
    let mut ctr: usize = 0;

    // Button edge-detection and debounce state
    let mut prev_left = false;
    let mut prev_right = false;
    let mut cd_left: u32 = 0;
    let mut cd_right: u32 = 0;

    // CENTER is hold-sensitive rather than edge-triggered: short press arms,
    // long press switches mode. `enter_consumed` suppresses the arm toggle on
    // release once a long press has already fired.
    let mut held_enter: u32 = 0;
    let mut enter_consumed = false;

    // UP/DOWN use hold-to-repeat rather than plain edge detection
    let mut held_up: u32 = 0;
    let mut held_down: u32 = 0;

    defmt::info!(
        "Torque bench ready. Wheel {}, diameter {}mm, Kt {}Nm/A, mode {}.",
        WHEEL_NAMES[active_wheel],
        WHEEL_DIAMETER_MM,
        MOTOR_KT_NM_PER_A,
        mode.name()
    );
    defmt::info!(
        "LEFT/RIGHT select wheel, UP/DOWN adjust setpoint ({}mA or {}mV per step, hold to repeat).",
        CURRENT_STEP_MA,
        VOLTAGE_STEP_MV
    );
    defmt::info!("CENTER: tap to arm/disarm, hold ~1s to switch CURRENT <-> VOLTAGE.");

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
                    "Armed for {}s. Output disarmed, let the motor cool.",
                    ARMED_TIMEOUT_S
                );
            }
        }

        //////////////
        //  inputs  //
        //////////////

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

        // CENTER: short press arms / disarms, long press switches control mode
        if now_enter {
            held_enter += 1;

            if held_enter == BTN_LONG_PRESS && !enter_consumed {
                enter_consumed = true;
                armed = false;
                setpoint_ma = 0;
                setpoint_mv = 0;
                mode = mode.toggled();
                defmt::info!(
                    "Control mode: {}. Setpoints zeroed, output disarmed.",
                    mode.name()
                );
            }
        } else {
            if held_enter > 0 && !enter_consumed {
                armed = !armed;
                if armed {
                    armed_at = Instant::now();
                    match mode {
                        ControlMode::Current => defmt::info!(
                            "ARMED. {} CURRENT {}mA. Predicted torque {}mNm ({}N at wheel).",
                            WHEEL_NAMES[active_wheel],
                            setpoint_ma,
                            current_ma_to_torque_mnm(setpoint_ma as f32),
                            current_ma_to_wheel_force_n(setpoint_ma as f32)
                        ),
                        ControlMode::Voltage => defmt::info!(
                            "ARMED. {} VOLTAGE {}mV open loop -> {}mA at stall.",
                            WHEEL_NAMES[active_wheel],
                            setpoint_mv,
                            voltage_mv_to_stall_current_ma(setpoint_mv as f32)
                        ),
                    }
                } else {
                    defmt::info!("DISARMED.");
                }
            }

            held_enter = 0;
            enter_consumed = false;
        }

        // RIGHT / LEFT: select active wheel. Always disarms - the setpoint
        // should be re-confirmed for the newly selected wheel.
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
            let (step, limit, cur) = match mode {
                ControlMode::Current => (CURRENT_STEP_MA, MAX_SETPOINT_MA, setpoint_ma),
                ControlMode::Voltage => (VOLTAGE_STEP_MV, MAX_SETPOINT_MV, setpoint_mv),
            };

            let next = if step_up {
                cur.saturating_add(step)
            } else {
                cur.saturating_sub(step)
            }
            .clamp(-limit, limit);

            if next != cur {
                let disarmed = if armed { "" } else { " (output disarmed)" };
                match mode {
                    ControlMode::Current => {
                        setpoint_ma = next;
                        defmt::info!(
                            "Setpoint {}mA -> predicted torque {}mNm ({}N at wheel){}",
                            next,
                            current_ma_to_torque_mnm(next as f32),
                            current_ma_to_wheel_force_n(next as f32),
                            disarmed
                        );
                    }
                    ControlMode::Voltage => {
                        setpoint_mv = next;
                        let stall_ma = voltage_mv_to_stall_current_ma(next as f32);
                        defmt::info!(
                            "Setpoint {}mV open loop -> {}mA at stall, torque {}mNm ({}N at wheel){}",
                            next,
                            stall_ma,
                            current_ma_to_torque_mnm(stall_ma),
                            current_ma_to_wheel_force_n(stall_ma),
                            disarmed
                        );
                    }
                }
            }
        }

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
                mode: mode as u8,
                setpoint_ma,
                setpoint_mv,
                bus_current_ma: m.read_current_filt_ma(),
                corrected_current_ma: m.read_current_duty_corrected_ma(),
                sync_current_ma: m.read_current_unfilt_ma(),
                duty_arr: m.read_cs_duty_arr(),
                wheel_vel_drads: (m.read_rads() * 10.0) as i16,
                bus_voltage_mv: (m.read_vbus_voltage() * 1000.0) as u16,
            };
            sample_pub.publish_immediate(sample);

            row_log_ctr += 1;
            if row_log_ctr >= row_log_divisor {
                row_log_ctr = 0;
                log_row(
                    &motors[active_wheel],
                    active_wheel,
                    mode,
                    setpoint_ma,
                    setpoint_mv,
                    armed,
                );
            }
        }

        // Apply setpoints: the active motor gets the setpoint only while armed,
        // everything else is held at zero. The motion type is rewritten every
        // tick so a mode switch takes effect on the next command without any
        // separate handshake.
        for (i, motor) in motors.iter_mut().enumerate() {
            let live = i == active_wheel && armed;
            match mode {
                ControlMode::Current => {
                    motor.set_motion_type(CcmMotionControlType::CCM_MCT_CURRENT);
                    motor.set_current_setpoint(if live { setpoint_ma } else { 0 });
                    motor.set_setpoint(0.0);
                }
                ControlMode::Voltage => {
                    motor.set_motion_type(CcmMotionControlType::CCM_MCT_VOLTAGE_OPENLOOP);
                    motor.set_setpoint(if live { setpoint_mv as f32 } else { 0.0 });
                    motor.set_current_setpoint(0);
                }
            }
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

/// Bus power, duty-corrected, and sync-sampled current side by side, each
/// converted to torque and to force at the wheel radius, plus how far the
/// duty-corrected estimate sits from the sync-sampled ground truth.
fn log_row(
    m: &TorqueTestMotor,
    wheel: usize,
    mode: ControlMode,
    setpoint_ma: i16,
    setpoint_mv: i16,
    armed: bool,
) {
    // In voltage mode there is no commanded current, so the err_sp columns are
    // taken against what the voltage should draw at stall. That is only
    // meaningful with the shaft held; once it turns, back-EMF makes the
    // reference read high.
    let sp = match mode {
        ControlMode::Current => setpoint_ma as f32,
        ControlMode::Voltage => voltage_mv_to_stall_current_ma(setpoint_mv as f32),
    };

    let vbus = m.read_vbus_voltage();
    let bus_current = m.read_current_filt_ma() as f32;
    let corrected_current = m.read_current_duty_corrected_ma() as f32;
    let sync_current = m.read_current_unfilt_ma() as f32;
    let sync_valid = m.read_cs_sync_sampling_enabled();

    let bus_power_mw = vbus * bus_current;

    defmt::info!(
        "== {} {} {} sp {}mA / {}mV (ref {}mA)  duty {}% (D_arr {}, corr_valid {})  vbus {}V  vel {}rad/s  sync {}",
        WHEEL_NAMES[wheel],
        mode.name(),
        if armed { "ARMED" } else { "idle" },
        setpoint_ma,
        setpoint_mv,
        sp,
        m.read_cs_duty() * 100.0,
        m.read_cs_duty_arr(),
        m.read_cs_duty_correction_valid(),
        vbus,
        m.read_rads(),
        sync_valid
    );

    defmt::info!(
        "   bus power     {}mW  I {}mA (naive, D*I_phase)  torque {}mNm  force {}N  err_sp {}%",
        bus_power_mw,
        bus_current,
        current_ma_to_torque_mnm(bus_current),
        current_ma_to_wheel_force_n(bus_current),
        err_pct(bus_current, sp)
    );

    defmt::info!(
        "   corrected     I {}mA  torque {}mNm  force {}N  err_sp {}%",
        corrected_current,
        current_ma_to_torque_mnm(corrected_current),
        current_ma_to_wheel_force_n(corrected_current),
        err_pct(corrected_current, sp)
    );

    defmt::info!(
        "   sync sample   I {}mA  torque {}mNm  force {}N  err_sp {}%",
        sync_current,
        current_ma_to_torque_mnm(sync_current),
        current_ma_to_wheel_force_n(sync_current),
        err_pct(sync_current, sp)
    );

    if sync_valid {
        defmt::info!(
            "   corrected vs sync  {}%  (duty-corrected relative to sync-sampled ground truth)",
            pct_diff(corrected_current, sync_current)
        );
    }
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
