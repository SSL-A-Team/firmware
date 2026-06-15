#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

use ateam_common_packets::bindings::{CcmMotionControlType, CcmTelemetry};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart,
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Input, Pull},
    interrupt, pac::Interrupt, peripherals, usart::Uart,
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
const MAX_RX_PACKET_SIZE: usize = 80;
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

static MOTOR_FB_PUBSUB: PubSubChannel<CriticalSectionRawMutex, CcmTelemetry, 3, 1, 1> =
    PubSubChannel::new();

// Standard coordinate convention: FL=0, BL=1, BR=2, FR=3
// Clockwise physical order: FL(0) -> FR(3) -> BR(2) -> BL(1) -> FL(0)
const WHEEL_NAMES: [&str; 4] = ["FL", "BL", "BR", "FR"];
const CW_NEXT:  [usize; 4] = [3, 0, 1, 2]; // FL->FR, BL->FL, BR->BL, FR->BR
const CCW_NEXT: [usize; 4] = [1, 2, 3, 0]; // FL->BL, BL->BR, BR->FR, FR->FL
const PULLEY_RADII_MM: [f32; 3] = [6.0, 10.0, 20.0];

type TorqueTestMotor = CurrentControlledMotor<
    'static,
    MAX_RX_PACKET_SIZE,
    MAX_TX_PACKET_SIZE,
    RX_BUF_DEPTH,
    TX_BUF_DEPTH,
    false,
>;

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
    let btn_left  = Input::new(p.PE12, Pull::Up);
    let btn_right = Input::new(p.PE13, Pull::Up);
    let btn_up    = Input::new(p.PE14, Pull::Up);
    let btn_down  = Input::new(p.PE15, Pull::Up);

    let initial_uart_config = stm32_interface::get_bootloader_uart_config();

    // Uart::new(uart, rx_pin, tx_pin, irqs, tx_dma, rx_dma, config)
    let fl_uart = Uart::new(p.UART7,   p.PF6, p.PF7, SystemIrqs, p.DMA1_CH0, p.DMA1_CH1, initial_uart_config).unwrap();
    let fr_uart = Uart::new(p.USART3,  p.PD9, p.PD8, SystemIrqs, p.DMA1_CH6, p.DMA1_CH7, initial_uart_config).unwrap();
    let br_uart = Uart::new(p.USART6,  p.PC7, p.PC6, SystemIrqs, p.DMA1_CH4, p.DMA1_CH5, initial_uart_config).unwrap();
    let bl_uart = Uart::new(p.USART10, p.PE2, p.PE3, SystemIrqs, p.DMA1_CH2, p.DMA1_CH3, initial_uart_config).unwrap();

    FRONT_LEFT_IDLE_BUFFERED_UART.init();
    FRONT_RIGHT_IDLE_BUFFERED_UART.init();
    BACK_RIGHT_IDLE_BUFFERED_UART.init();
    BACK_LEFT_IDLE_BUFFERED_UART.init();

    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, FRONT_LEFT,  fl_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, FRONT_RIGHT, fr_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, BACK_RIGHT,  br_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, BACK_LEFT,   bl_uart);

    // Motors indexed by coordinate convention: FL=0, BL=1, BR=2, FR=3
    let mut motors: [TorqueTestMotor; 4] = [
        TorqueTestMotor::new_from_pins(
            &FRONT_LEFT_IDLE_BUFFERED_UART,
            FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PF5.into(), p.PF4.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
        TorqueTestMotor::new_from_pins(
            &BACK_LEFT_IDLE_BUFFERED_UART,
            BACK_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            BACK_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PE5.into(), p.PE4.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
        TorqueTestMotor::new_from_pins(
            &BACK_RIGHT_IDLE_BUFFERED_UART,
            BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PG7.into(), p.PG8.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
        TorqueTestMotor::new_from_pins(
            &FRONT_RIGHT_IDLE_BUFFERED_UART,
            FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(),
            FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(),
            p.PB12.into(), p.PB13.into(),
            CURRENT_CONTROLLED_WHEEL_IMAGE,
        ),
    ];

    let torque_data_pub = MOTOR_FB_PUBSUB.publisher().expect("could not get motor data publisher");
    let usb_subscriber  = MOTOR_FB_PUBSUB.subscriber().expect("could not get motor data subscriber");

    defmt::info!("Setting up USB...");
    let mut usb_hw_config = embassy_stm32::usb::Config::default();
    usb_hw_config.vbus_detection = false;

    let ep_out_buffer: &'static mut [u8; 4096] = unsafe { &mut (*(&raw mut EP_OUT_BUFFER_CELL)) };
    let usb_driver = embassy_stm32::usb::Driver::new_fs(
        p.USB_OTG_HS, Irqs, p.PA12, p.PA11, ep_out_buffer, usb_hw_config,
    );

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer  = Some("A-Team");
    usb_config.product       = Some("Control Board");
    usb_config.serial_number = Some("12345678");

    let usb_state:         &'static mut State      = unsafe { &mut (*(&raw mut USB_STATE_CELL)) };
    let config_descriptor: &'static mut [u8; 256]  = unsafe { &mut (*(&raw mut CONFIG_DESCRIPTOR_CELL)) };
    let bos_descriptor:    &'static mut [u8; 256]  = unsafe { &mut (*(&raw mut BOS_DESCRIPTOR_CELL)) };
    let control_buf:       &'static mut [u8; 4096] = unsafe { &mut (*(&raw mut CONTROL_BUF_CELL)) };

    let mut usb_builder = embassy_usb::Builder::new(
        usb_driver, usb_config, config_descriptor, bos_descriptor, &mut [], control_buf,
    );
    let cdc_usb_class = CdcAcmClass::new(&mut usb_builder, usb_state, 64);
    let usb_device_driver = usb_builder.build();

    main_spawner.spawn(usb_ll_driver_task(usb_device_driver)).expect("failed to spawn USB driver task");
    main_spawner.spawn(usb_writer_task(cdc_usb_class, usb_subscriber)).expect("failed to spawn USB task");

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

    // Test state
    let mut active_wheel:     usize = 0; // 0=FL, 1=FR, 2=BR, 3=BL (clockwise)
    let mut pulley_radius_idx: usize = 0; // index into PULLEY_RADII_MM
    let mut test_running  = false;
    let mut ramp_start    = Instant::now();
    let mut moved_yet     = false;
    let mut curr_setpoint: i16 = 0;
    let mut last_seq_num:  u8 = 0;
    let mut ctr:           usize = 0;

    // Button edge-detection state (true = was pressed on previous tick)
    let mut prev_enter = false;
    let mut prev_left  = false;
    let mut prev_right = false;
    let mut prev_up    = false;
    let mut prev_down  = false;

    // Per-button debounce cooldown counters (500 µs ticks; 200 ticks = 100 ms)
    const BTN_COOLDOWN: u32 = 200;
    let mut cd_enter: u32 = 0;
    let mut cd_left:  u32 = 0;
    let mut cd_right: u32 = 0;
    let mut cd_up:    u32 = 0;
    let mut cd_down:  u32 = 0;

    defmt::info!(
        "Ready. Active wheel: {}, Pulley: {}mm. Press CENTER to start a test round.",
        WHEEL_NAMES[active_wheel],
        PULLEY_RADII_MM[pulley_radius_idx]
    );

    let mut ticker = Ticker::every(Duration::from_micros(500));
    loop {
        // Process incoming motor packets for all motors
        for motor in motors.iter_mut() {
            motor.process_packets();
        }

        // Forward active motor telemetry to USB
        let cur_seq = motors[active_wheel].get_latest_state_seqnum();
        if cur_seq != last_seq_num {
            torque_data_pub.publish_immediate(motors[active_wheel].get_latest_state());
            last_seq_num = cur_seq;
        }

        // Debounce cooldown countdown
        if cd_enter > 0 { cd_enter -= 1; }
        if cd_left  > 0 { cd_left  -= 1; }
        if cd_right > 0 { cd_right -= 1; }
        if cd_up    > 0 { cd_up    -= 1; }
        if cd_down  > 0 { cd_down  -= 1; }

        let now_enter = btn_enter.is_low();
        let now_left  = btn_left.is_low();
        let now_right = btn_right.is_low();
        let now_up    = btn_up.is_low();
        let now_down  = btn_down.is_low();

        // CENTER: start/restart a test round
        if now_enter && !prev_enter && cd_enter == 0 {
            cd_enter = BTN_COOLDOWN;
            test_running  = true;
            moved_yet     = false;
            curr_setpoint = 0;
            ramp_start    = Instant::now();
            defmt::info!(
                "Test round started. Wheel: {}, Pulley radius: {}mm",
                WHEEL_NAMES[active_wheel],
                PULLEY_RADII_MM[pulley_radius_idx]
            );
        }

        // RIGHT: advance active wheel clockwise (FL->FR->BR->BL->FL)
        if now_right && !prev_right && cd_right == 0 {
            cd_right = BTN_COOLDOWN;
            active_wheel  = CW_NEXT[active_wheel];
            test_running  = false;
            moved_yet     = false;
            curr_setpoint = 0;
            last_seq_num  = 0;
            defmt::info!(
                "Active wheel: {} (clockwise). Press CENTER to start a test round.",
                WHEEL_NAMES[active_wheel]
            );
        }

        // LEFT: advance active wheel counter-clockwise (FL->BL->BR->FR->FL)
        if now_left && !prev_left && cd_left == 0 {
            cd_left = BTN_COOLDOWN;
            active_wheel  = CCW_NEXT[active_wheel];
            test_running  = false;
            moved_yet     = false;
            curr_setpoint = 0;
            last_seq_num  = 0;
            defmt::info!(
                "Active wheel: {} (counter-clockwise). Press CENTER to start a test round.",
                WHEEL_NAMES[active_wheel]
            );
        }

        // UP: increase pulley radius
        if now_up && !prev_up && cd_up == 0 {
            cd_up = BTN_COOLDOWN;
            if pulley_radius_idx < PULLEY_RADII_MM.len() - 1 {
                pulley_radius_idx += 1;
            }
            defmt::info!("Pulley radius set to {}mm", PULLEY_RADII_MM[pulley_radius_idx]);
        }

        // DOWN: decrease pulley radius
        if now_down && !prev_down && cd_down == 0 {
            cd_down = BTN_COOLDOWN;
            if pulley_radius_idx > 0 {
                pulley_radius_idx -= 1;
            }
            defmt::info!("Pulley radius set to {}mm", PULLEY_RADII_MM[pulley_radius_idx]);
        }

        prev_enter = now_enter;
        prev_left  = now_left;
        prev_right = now_right;
        prev_up    = now_up;
        prev_down  = now_down;

        // Current ramp: ramps negative (reverse direction); only active when test is running
        if test_running && !moved_yet {
            let elapsed_ms = (Instant::now() - ramp_start).as_millis();
            let prev_sp = curr_setpoint;
            curr_setpoint = -((elapsed_ms / 250) as i16);
            if curr_setpoint < -500 { curr_setpoint = -500; }
            if curr_setpoint != prev_sp {
                defmt::info!("Current setpoint: {}mA", -curr_setpoint);
            }
        } else {
            curr_setpoint = 0;
        }

        // Detect wheel movement on the active motor (negative velocity for reverse direction)
        if test_running && !moved_yet && motors[active_wheel].read_rads() < -(3.14 * 4.0) {
            moved_yet = true;
            let break_current = -curr_setpoint; // report as positive
            curr_setpoint = 0;
            test_running  = false;
            let radius_mm     = PULLEY_RADII_MM[pulley_radius_idx];
            let torque_signal = break_current as f32 * radius_mm;
            defmt::info!(
                "Movement detected on {}! Break-away current: {}mA, Pulley radius: {}mm, Torque signal: {}mA*mm",
                WHEEL_NAMES[active_wheel],
                break_current,
                radius_mm,
                torque_signal
            );
        }

        // Apply setpoints: active motor gets the ramp, all others stay at zero
        for (i, motor) in motors.iter_mut().enumerate() {
            let sp = if i == active_wheel { curr_setpoint } else { 0i16 };
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
    mut packet_sub: Subscriber<'static, CriticalSectionRawMutex, CcmTelemetry, 3, 1, 1>,
) {
    loop {
        defmt::info!("USB task - waiting connection...");
        usb_class.wait_connection().await;
        defmt::info!("Connected");

        loop {
            let feedback_data_packet = packet_sub.next_message_pure().await;

            let struct_bytes = unsafe {
                core::slice::from_raw_parts(
                    (&feedback_data_packet as *const CcmTelemetry) as *const u8,
                    core::mem::size_of::<CcmTelemetry>(),
                )
            };

            defmt::info!("writing torque feedback packet to USB...");
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

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
