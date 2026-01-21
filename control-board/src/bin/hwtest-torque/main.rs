#![no_std]
#![no_main]

#![feature(sync_unsafe_cell)]

use embassy_stm32::{peripherals::USB_OTG_HS, usb::{DmPin, DpPin, Driver, Instance}, Peri};
use ateam_common_packets::{bindings::{BasicControl, CurrentControlledMotor_MotionControlType, CurrentControlledMotor_Telemetry, KickRequest}, radio::DataPacket};
use ateam_lib_stm32::{drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart, uart::queue::IdleBufferedUart};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{interrupt, pac::Interrupt, usart::Uart, peripherals, bind_interrupts};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::{PubSubChannel, Subscriber}};

use defmt_rtt as _;

use ateam_control_board::{
    create_control_task, create_imu_task, create_io_task, get_system_config, include_external_cpp_bin, motor::CurrentControlledMotor, pins::{
        AccelDataPubSub, CommandsPubSub, GyroDataPubSub, KickerTelemetryPubSub, LedCommandPubSub,
        PowerTelemetryPubSub, TelemetryPubSub,
    }, robot_state::SharedRobotState, SystemIrqs
};

use embassy_time::{Duration, Ticker, Timer};
// provide embedded panic probe
use panic_probe as _;
use static_cell::{ConstStaticCell, StaticCell};

use embassy_usb::{class::cdc_acm::{CdcAcmClass, State}, UsbDevice};
use embassy_usb::driver::EndpointError;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static RADIO_UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

include_external_cpp_bin! {CURRENT_CONTROLLED_WHEEL_IMAGE, "wheel-torque.bin"}

const MAX_TX_PACKET_SIZE: usize = 80;
const TX_BUF_DEPTH: usize = 5;
const MAX_RX_PACKET_SIZE: usize = 80;
const RX_BUF_DEPTH: usize = 5;

static_idle_buffered_uart!(CCM_UART, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, false, #[link_section = ".axisram.buffers"]);

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

static MOTOR_FB_PUBSUB: PubSubChannel<CriticalSectionRawMutex, CurrentControlledMotor_Telemetry, 3, 1, 1> = PubSubChannel::new();

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

    let back_right_uart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        SystemIrqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        initial_motor_controller_uart_conifg,
    ).unwrap();

    CCM_UART_IDLE_BUFFERED_UART.init();

    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, CCM_UART, back_right_uart);

    //////////////////////////////
    //  setup pub sub channels  //
    //////////////////////////////
    
    let torque_data_pub = MOTOR_FB_PUBSUB.publisher().expect("could not get motor data publisher");
    let usb_subscriber = MOTOR_FB_PUBSUB.subscriber().expect("could not get motor data subscriber");

    ///////////////////
    //  start tasks  //
    ///////////////////
    
    defmt::info!("Setting up USB...");
    let mut usb_hw_config = embassy_stm32::usb::Config::default();
    usb_hw_config.vbus_detection = false;

    let ep_out_buffer: &'static mut [u8; 4096] = unsafe { &mut (*(&raw mut EP_OUT_BUFFER_CELL)) };


    // USB Driver::new_hs() call doesn't work. 
    let usb_driver = embassy_stm32::usb::Driver::new_fs(p.USB_OTG_HS, Irqs, p.PA12, p.PA11, ep_out_buffer, usb_hw_config);

    // Create embassy-usb Config
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("A-Team");
    usb_config.product = Some("Control Board");
    usb_config.serial_number = Some("12345678");

    let usb_state: &'static mut State = unsafe { &mut (*(&raw mut USB_STATE_CELL)) };

    let config_descriptor: &'static mut [u8; 256] = unsafe { &mut (*(&raw mut CONFIG_DESCRIPTOR_CELL)) };
    let bos_descriptor:  &'static mut [u8; 256] = unsafe { &mut (*(&raw mut BOS_DESCRIPTOR_CELL)) };
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

    main_spawner.spawn(usb_ll_driver_task(usb_device_driver)).expect("failed to spawn USB driver task");
    main_spawner.spawn(usb_writer_task(cdc_usb_class, usb_subscriber)).expect("failed to spawn USB task");

    /////////////////////////////
    //  main task motor logic  //
    /////////////////////////////

    // block main task for trace level usb debugging
    // loop {
    //     Timer::after_millis(100).await;
    // }

    let mut ccm = CurrentControlledMotor::new_from_pins(&CCM_UART_IDLE_BUFFERED_UART, CCM_UART_IDLE_BUFFERED_UART.get_uart_read_queue(), CCM_UART_IDLE_BUFFERED_UART.get_uart_write_queue(), p.PG7.into(), p.PG8.into(), CURRENT_CONTROLLED_WHEEL_IMAGE);

    defmt::info!("flasing motor...");

    let res = ccm.init_default_firmware_image(true).await;

    if res.is_ok() {
        defmt::info!("motor flashed.");
    } else {
        defmt::error!("motor failed to flash!");
    }


    ccm.set_motion_type(CurrentControlledMotor_MotionControlType::CCM_MCT_VOLTAGE_OPENLOOP);
    ccm.set_setpoint(2500.0);
    ccm.set_telemetry_enabled(true);
    ccm.set_motion_enabled(true);


    ccm.leave_reset().await;

    let mut last_seq_num = 0;

    let mut ctr = 0;
    let mut ticker = Ticker::every(Duration::from_micros(500));
    loop {
        ccm.process_packets();

        let v = ccm.read_vbus_voltage();
        let i = ccm.read_current_estimate_ma();
        let w = ccm.read_rads();

        let cur_seq_num = ccm.get_latest_state_seqnum();
        if cur_seq_num != last_seq_num {
            torque_data_pub.publish_immediate(ccm.get_latest_state());
            // defmt::info!("got a unique state update, sending to usb task");

            last_seq_num = cur_seq_num;
        }

        if ctr > 19 {
            // defmt::info!("vrail: {}, current: {}, vel: {}", v, i, w);
            ctr = 0;
        } else {
            ctr += 1;
        }

        ccm.send_motion_command();

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn usb_ll_driver_task(mut usb_device:  UsbDevice<'static, Driver<'static, USB_OTG_HS>>) {
    loop {
        defmt::info!("starting USB ll driver core.");

        usb_device.run().await;

        defmt::panic!("usb device driver task returned!");
    }
}

#[embassy_executor::task]
async fn usb_writer_task(mut usb_class: CdcAcmClass<'static, Driver<'static, USB_OTG_HS>>, mut packet_sub: Subscriber<'static, CriticalSectionRawMutex, CurrentControlledMotor_Telemetry, 3, 1, 1>) {
    loop {
        defmt::info!("USB task - waiting connection...");
        usb_class.wait_connection().await;
        defmt::info!("Connected");

        loop {
            let feedback_data_packet = packet_sub.next_message_pure().await;

            let struct_bytes = unsafe {
                core::slice::from_raw_parts(
                (&feedback_data_packet as *const CurrentControlledMotor_Telemetry) as *const u8,
                core::mem::size_of::<CurrentControlledMotor_Telemetry>(),
            )
            };

            defmt::info!("writing torque feedback packet to USB...");
            let res = usb_class.write_packet(struct_bytes).await;
            if res.is_err() {
                match res.err().unwrap() {
                    EndpointError::BufferOverflow => {
                        defmt::error!("USB transmit buffer overflowed");
                    },
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