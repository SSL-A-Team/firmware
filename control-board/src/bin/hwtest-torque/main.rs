#![no_std]
#![no_main]

#![feature(sync_unsafe_cell)]

use ateam_common_packets::{bindings::{BasicControl, CurrentControlledMotor_MotionControlType, KickRequest}, radio::DataPacket};
use ateam_lib_stm32::{drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart, uart::queue::IdleBufferedUart};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{interrupt, pac::Interrupt, usart::Uart};
use embassy_sync::pubsub::PubSubChannel;

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
use static_cell::ConstStaticCell;

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

    let mut ccm = CurrentControlledMotor::new_from_pins(&CCM_UART_IDLE_BUFFERED_UART, CCM_UART_IDLE_BUFFERED_UART.get_uart_read_queue(), CCM_UART_IDLE_BUFFERED_UART.get_uart_write_queue(), p.PG7.into(), p.PG8.into(), CURRENT_CONTROLLED_WHEEL_IMAGE);


    ///////////////////
    //  start tasks  //
    ///////////////////
    
    defmt::info!("flasing motor...");

    let res = ccm.init_default_firmware_image(true).await;

    if res.is_ok() {
        defmt::info!("motor flashed.");
    } else {
        defmt::error!("motor failed to flash!");
    }


    ccm.set_motion_type(CurrentControlledMotor_MotionControlType::CCM_MCT_MOTOR_OFF);
    ccm.set_setpoint(0.0);
    ccm.set_telemetry_enabled(true);
    ccm.set_motion_enabled(true);


    ccm.leave_reset().await;

    let mut ctr = 0;
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        ccm.process_packets();

        let v = ccm.read_vbus_voltage();
        let i = ccm.read_current_estimate_ma();
        let w = ccm.read_rads();

        if ctr > 9 {
            defmt::info!("vrail: {}, current: {}, vel: {}", v, i, w);
            ctr = 0;
        } else {
            ctr += 1;
        }

        ccm.send_motion_command();

        ticker.next().await;
    }
}
