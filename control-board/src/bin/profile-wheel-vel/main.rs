#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

use ateam_common_packets::bindings::{CcmMotionControlType, CcmTelemetry};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart,
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{bind_interrupts, interrupt, pac::Interrupt, peripherals, usart::Uart};
use embassy_stm32::{peripherals::USB_OTG_HS, usb::Driver};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{PubSubChannel, Subscriber},
};

use defmt_rtt as _;

use ateam_control_board::{
    get_system_config, include_external_cpp_bin, motor::CurrentControlledMotor,
    robot_state::SharedRobotState, SystemIrqs,
};

use embassy_time::{Duration, Ticker};
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

use embassy_usb::driver::EndpointError;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    UsbDevice,
};

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static RADIO_UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

include_external_cpp_bin! {CURRENT_CONTROLLED_WHEEL_IMAGE, "wheel-torque.bin"}

const MAX_TX_PACKET_SIZE: usize = 80;
const TX_BUF_DEPTH: usize = 5;
const MAX_RX_PACKET_SIZE: usize = 80;
const RX_BUF_DEPTH: usize = 5;

static_idle_buffered_uart!(CCM_FL_UART, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(CCM_BL_UART, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(CCM_BR_UART, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(CCM_FR_UART, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);

// Create embassy-usb DeviceBuilder using the driver and config.
// It needs some buffers for building the descriptors.
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

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CORDIC() {
    RADIO_UART_QUEUE_EXECUTOR.on_interrupt();
}

static MOTOR_FB_PUBSUB: PubSubChannel<CriticalSectionRawMutex, CcmTelemetry, 3, 1, 1> =
    PubSubChannel::new();

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    // init system
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("embassy HAL configured.");

    let robot_state = ROBOT_STATE.take();

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P7,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    ////////////////////////////////////
    //  create single motor instance  //
    ////////////////////////////////////

    let initial_motor_controller_uart_conifg = stm32_interface::get_bootloader_uart_config();

    let front_left_uart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        SystemIrqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();

    let back_left_uart = Uart::new(
        p.USART10,
        p.PE2,
        p.PE3,
        SystemIrqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();

    let back_right_uart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        SystemIrqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();

    let front_right_uart = Uart::new(
        p.USART3,
        p.PD9,
        p.PD8,
        SystemIrqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();

    CCM_FL_UART_IDLE_BUFFERED_UART.init();
    CCM_BL_UART_IDLE_BUFFERED_UART.init();
    CCM_BR_UART_IDLE_BUFFERED_UART.init();
    CCM_FR_UART_IDLE_BUFFERED_UART.init();

    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, CCM_FL_UART, front_left_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, CCM_BL_UART, back_left_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, CCM_BR_UART, back_right_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, CCM_FR_UART, front_right_uart);

    let mut fl_ccm = CurrentControlledMotor::new_from_pins(
        &CCM_FL_UART_IDLE_BUFFERED_UART,
        CCM_FL_UART_IDLE_BUFFERED_UART.get_uart_read_queue(),
        CCM_FL_UART_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PF5.into(),
        p.PF4.into(),
        CURRENT_CONTROLLED_WHEEL_IMAGE,
    );
    let mut bl_ccm = CurrentControlledMotor::new_from_pins(
        &CCM_BL_UART_IDLE_BUFFERED_UART,
        CCM_BL_UART_IDLE_BUFFERED_UART.get_uart_read_queue(),
        CCM_BL_UART_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PE5.into(),
        p.PE4.into(),
        CURRENT_CONTROLLED_WHEEL_IMAGE,
    );
    let mut br_ccm = CurrentControlledMotor::new_from_pins(
        &CCM_BR_UART_IDLE_BUFFERED_UART,
        CCM_BR_UART_IDLE_BUFFERED_UART.get_uart_read_queue(),
        CCM_BR_UART_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PG7.into(),
        p.PG8.into(),
        CURRENT_CONTROLLED_WHEEL_IMAGE,
    );
    let mut fr_ccm = CurrentControlledMotor::new_from_pins(
        &CCM_FR_UART_IDLE_BUFFERED_UART,
        CCM_FR_UART_IDLE_BUFFERED_UART.get_uart_read_queue(),
        CCM_FR_UART_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PB12.into(),
        p.PB13.into(),
        CURRENT_CONTROLLED_WHEEL_IMAGE,
    );

    //////////////////////////////
    //  setup pub sub channels  //
    //////////////////////////////

    let torque_data_pub = MOTOR_FB_PUBSUB
        .publisher()
        .expect("could not get motor data publisher");
    let usb_subscriber = MOTOR_FB_PUBSUB
        .subscriber()
        .expect("could not get motor data subscriber");

    ///////////////////
    //  start tasks  //
    ///////////////////

    defmt::info!("Setting up USB...");
    let mut usb_hw_config = embassy_stm32::usb::Config::default();
    usb_hw_config.vbus_detection = false;

    let ep_out_buffer: &'static mut [u8; 4096] = unsafe { &mut (*(&raw mut EP_OUT_BUFFER_CELL)) };

    // USB Driver::new_hs() call doesn't work.
    let usb_driver = embassy_stm32::usb::Driver::new_fs(
        p.USB_OTG_HS,
        Irqs,
        p.PA12,
        p.PA11,
        ep_out_buffer,
        usb_hw_config,
    );

    // Create embassy-usb Config
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
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create classes on the builder.
    let cdc_usb_class = CdcAcmClass::new(&mut usb_builder, usb_state, 64);

    // Build the builder.
    let usb_device_driver = usb_builder.build();

    main_spawner
        .spawn(usb_ll_driver_task(usb_device_driver))
        .expect("failed to spawn USB driver task");
    main_spawner
        .spawn(usb_writer_task(cdc_usb_class, usb_subscriber))
        .expect("failed to spawn USB task");

    /////////////////////////////
    //  main task motor logic  //
    /////////////////////////////

    defmt::info!("flasing motor...");

    let res = fl_ccm.init_default_firmware_image(true).await;
    let res = bl_ccm.init_default_firmware_image(true).await;
    let res = br_ccm.init_default_firmware_image(true).await;
    let res = fr_ccm.init_default_firmware_image(true).await;

    if res.is_ok() {
        defmt::info!("motor flashed.");
    } else {
        defmt::error!("motor failed to flash!");
    }

    // ccm.set_motion_type(CcmMotionControlType::CCM_MCT_VOLTAGE_OPENLOOP);
    // ccm.set_setpoint(2500.0);
    fl_ccm.set_motion_type(CcmMotionControlType::CCM_MCT_VELOCITY_CURRENT);
    fl_ccm.set_current_setpoint(0);
    bl_ccm.set_motion_type(CcmMotionControlType::CCM_MCT_VELOCITY_CURRENT);
    bl_ccm.set_current_setpoint(0);
    br_ccm.set_motion_type(CcmMotionControlType::CCM_MCT_VELOCITY_CURRENT);
    br_ccm.set_current_setpoint(0);
    fr_ccm.set_motion_type(CcmMotionControlType::CCM_MCT_VELOCITY_CURRENT);
    fr_ccm.set_current_setpoint(0);

    fl_ccm.set_telemetry_enabled(true);
    fl_ccm.set_motion_enabled(true);
    bl_ccm.set_telemetry_enabled(true);
    bl_ccm.set_motion_enabled(true);
    br_ccm.set_telemetry_enabled(true);
    br_ccm.set_motion_enabled(true);
    fr_ccm.set_telemetry_enabled(true);
    fr_ccm.set_motion_enabled(true);

    fl_ccm.leave_reset().await;
    bl_ccm.leave_reset().await;
    br_ccm.leave_reset().await;
    fr_ccm.leave_reset().await;

    let mut last_seq_num = 0;

    let mut mv_cmd_counting_up = true;
    let mut mv_cmd = 0.0;

    let mut curr_ctr = 0;

    let mut ctr = 0;
    let mut ticker = Ticker::every(Duration::from_micros(500));
    loop {
        fl_ccm.process_packets();
        bl_ccm.process_packets();
        br_ccm.process_packets();
        fr_ccm.process_packets();

        let v = fl_ccm.read_vbus_voltage();
        let i_sp = fl_ccm.read_current_setpoint_ma();
        let i_ref = fl_ccm.read_current_estimate_ma();
        let w = fl_ccm.read_rads();
        let Vm = fl_ccm.read_vmotor_voltage_mv();

        let cur_seq_num = fl_ccm.get_latest_state_seqnum();
        if cur_seq_num != last_seq_num {
            torque_data_pub.publish_immediate(fl_ccm.get_latest_state());
            // defmt::info!("got a unique state update, sending to usb task");

            last_seq_num = cur_seq_num;
        }

        if ctr > 19 {
            defmt::info!(
                "motion control type: {}",
                fl_ccm.get_latest_state().motion_control_type
            );
            defmt::info!(
                "vrail: {}, Isp: {}, Iref: {}, vel: {}, Vmv: {}",
                v,
                i_sp,
                i_ref,
                w,
                Vm
            );
            ctr = 0;
        } else {
            ctr += 1;
        }

        if curr_ctr > 1000 {
            curr_ctr = 0
        } else if curr_ctr > 500 {
            fl_ccm.set_current_setpoint(40);
            bl_ccm.set_current_setpoint(40);
            br_ccm.set_current_setpoint(40);
            fr_ccm.set_current_setpoint(40);

            fl_ccm.set_setpoint(12.0);
            bl_ccm.set_setpoint(12.0);
            br_ccm.set_setpoint(12.0);
            fr_ccm.set_setpoint(12.0);
        } else {
            fl_ccm.set_current_setpoint(-40);
            bl_ccm.set_current_setpoint(-40);
            br_ccm.set_current_setpoint(-40);
            fr_ccm.set_current_setpoint(-40);

            fl_ccm.set_setpoint(-12.0);
            bl_ccm.set_setpoint(-12.0);
            br_ccm.set_setpoint(-12.0);
            fr_ccm.set_setpoint(-12.0);
        }

        curr_ctr += 1;

        // if mv_cmd_counting_up {
        //     mv_cmd += 0.1;
        // } else {
        //     mv_cmd -= 0.1;
        // }

        // if mv_cmd_counting_up && mv_cmd > 12500.0 {
        //     mv_cmd_counting_up = false;
        // }

        // if !mv_cmd_counting_up && mv_cmd < 1.0 {
        //     mv_cmd_counting_up = true;
        // }

        // ccm.set_setpoint(mv_cmd);

        fl_ccm.send_motion_command();
        bl_ccm.send_motion_command();
        br_ccm.send_motion_command();
        fr_ccm.send_motion_command();

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
