#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(async_closure)]

use core::f32::consts::PI;

use defmt_rtt as _;
use defmt::*;
use embassy_stm32::{
    self,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    gpio::{Level, OutputOpenDrain, Pull, Speed, Output},
    peripherals::{DMA1_CH0, DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7},
    peripherals::{USART3, UART4, UART5, UART7},
    usart::{Uart, Parity}, time::mhz,
};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;
use nalgebra::{Vector4, Vector3};
use panic_probe as _;
use static_cell::StaticCell;

use ateam_control_board::{
    stm32_interface::{Stm32Interface, self},
    queue::Buffer,
    uart_queue::{UartReadQueue, UartWriteQueue},
    include_external_cpp_bin, stspin_motor::WheelMotor, robot_model::{RobotConstants, self, RobotModel},
};

include_external_cpp_bin!{STEVAL3204_DRIB_POTCTRL_FW_IMG, "dev3204-drib-potctrl.bin"}
include_external_cpp_bin!{STEVAL3204_DRIB_FW_IMG, "dev3204-drib.bin"}
include_external_cpp_bin!{STEVAL3204_WHEEL_FW_IMG, "dev3204-wheel.bin"}


// motor pinout
// FrontRight Wheel - UART5  - tx pb6,  pb12,    boot pb1,  rst pb2, DMA0/1
// FrontLeft Wheel  - UART7  - tx pf7,  rx pf6,  boot pg2,  rst pg3, DMA2/3
// BackLeft Wheel   - UART4  - tx pd1,  rx pd0,  boot pg0,  rst pg1, DMA4/5
// BackRight Wheel  - USART3 - tx pb10, rx pb11, boot pf4,  rst pa3, DMA6/7

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

// buffers for front right
#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] = [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_TX: UartWriteQueue<UART5, DMA1_CH0, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] = [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_RX: UartReadQueue<UART5, DMA1_CH1, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_RX });

// buffers for front left
#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] = [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_TX: UartWriteQueue<UART7, DMA1_CH2, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] = [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_RX: UartReadQueue<UART7, DMA1_CH3, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_RX });

// buffers for back left
#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] = [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_LEFT_QUEUE_TX: UartWriteQueue<UART4, DMA1_CH4, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] = [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_LEFT_QUEUE_RX: UartReadQueue<UART4, DMA1_CH5, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_RX });

// buffers for back right
#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] = [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_TX: UartWriteQueue<USART3, DMA1_CH6, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] = [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_RX: UartReadQueue<USART3, DMA1_CH7, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_RX });

// executor queue
static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

const WHEEL_ANGLES_DEG: Vector4<f32> = Vector4::new(30.0, 150.0, 225.0, 315.0);
const WHEEL_RADIUS_M: f32 = 0.049 / 2.0; // wheel dia 49mm
const WHEEL_DISTANCE_TO_ROBOT_CENTER_M: f32 = 0.085; // 85mm from center of wheel body to center of robot

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    // setup system clocks
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    // place the stspins in reset as early as possible just in case we have lingering state
    // that wants to spin motors
    let front_right_reset_pin = OutputOpenDrain::new(p.PB2, Level::Low, Speed::Medium, Pull::None); // reset active
    let front_left_reset_pin = OutputOpenDrain::new(p.PG3, Level::Low, Speed::Medium, Pull::None); // reset active
    let back_left_reset_pin = OutputOpenDrain::new(p.PG1, Level::Low, Speed::Medium, Pull::None); // reset active
    let back_right_reset_pin = OutputOpenDrain::new(p.PA3, Level::Low, Speed::Medium, Pull::None); // reset active

    // setup the async executor
    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    // setup leds
    let mut grn_led = Output::new(p.PB0, Level::High, Speed::Medium);
    let mut ylw_led = Output::new(p.PE1, Level::High, Speed::Medium);
    let mut red_led = Output::new(p.PB14, Level::High, Speed::Medium);
    ylw_led.set_low();
    red_led.set_low();


    ///////////////////////////////
    //  initialize peripherials  //
    ///////////////////////////////
    
    let wheel_firmware_image = STEVAL3204_WHEEL_FW_IMG;

    /* FRONT RIGHT WHEEL - UART5 */
    // FrontRight Wheel - UART5  - tx pb6,  rx pb12,    boot pb1,  rst pb2

    // front right IO
    let front_right_uart_config = stm32_interface::get_bootloader_uart_config();
    let front_right_int = interrupt::take!(UART5);
    let front_right_usart = Uart::new(p.UART5, p.PB12, p.PB6, front_right_int, p.DMA1_CH0, p.DMA1_CH1, front_right_uart_config);
    let (front_right_tx, front_right_rx) = front_right_usart.split();
    let front_right_boot0_pin = Output::new(p.PB1, Level::Low, Speed::Medium); // boot0 not active

    // register front right uart primitives with executor
    spawner.spawn(FRONT_RIGHT_QUEUE_RX.spawn_task(front_right_rx)).unwrap();
    spawner.spawn(FRONT_RIGHT_QUEUE_TX.spawn_task(front_right_tx)).unwrap();

    // initialize the wheel
    let front_right_stm32_interface = Stm32Interface::new(&FRONT_RIGHT_QUEUE_RX, &FRONT_RIGHT_QUEUE_TX, Some(front_right_boot0_pin), Some(front_right_reset_pin));
    let mut front_right_motor = WheelMotor::new(front_right_stm32_interface, wheel_firmware_image);


    /* FRONT LEFT WHEEL - UART7 */
    // FrontLeft Wheel  - UART7  - tx pf7,  rx pf6,  boot pg2,  rst pg3

    // front left IO
    let front_left_uart_config = stm32_interface::get_bootloader_uart_config();
    let front_left_int = interrupt::take!(UART7);
    let front_left_usart = Uart::new(p.UART7, p.PF6, p.PF7, front_left_int, p.DMA1_CH2, p.DMA1_CH3, front_left_uart_config);
    let (front_left_tx, front_left_rx) = front_left_usart.split();
    let front_left_boot0_pin = Output::new(p.PG2, Level::Low, Speed::Medium); // boot0 not active

    // register front left uart primitives with executor
    spawner.spawn(FRONT_LEFT_QUEUE_RX.spawn_task(front_left_rx)).unwrap();
    spawner.spawn(FRONT_LEFT_QUEUE_TX.spawn_task(front_left_tx)).unwrap();

    // initialize the wheel
    let front_left_stm32_interface = Stm32Interface::new(&FRONT_LEFT_QUEUE_RX, &FRONT_LEFT_QUEUE_TX, Some(front_left_boot0_pin), Some(front_left_reset_pin));
    let mut front_left_motor = WheelMotor::new(front_left_stm32_interface, wheel_firmware_image);


    /* BACK LEFT WHEEL */
    // BackLeft Wheel   - UART4  - tx pd1,  rx pd0,  boot pg0,  rst pg1

    // back left IO
    let back_left_uart_config = stm32_interface::get_bootloader_uart_config();
    let back_left_int = interrupt::take!(UART4);
    let back_left_usart = Uart::new(p.UART4, p.PD0, p.PD1, back_left_int, p.DMA1_CH4, p.DMA1_CH5, back_left_uart_config);
    let (back_left_tx, back_left_rx) = back_left_usart.split();
    let back_left_boot0_pin = Output::new(p.PG0, Level::Low, Speed::Medium); // boot0 not active

    // register front left uart primitives with executor
    spawner.spawn(BACK_LEFT_QUEUE_RX.spawn_task(back_left_rx)).unwrap();
    spawner.spawn(BACK_LEFT_QUEUE_TX.spawn_task(back_left_tx)).unwrap();

    // initialize the wheel
    let back_left_stm32_interface = Stm32Interface::new(&BACK_LEFT_QUEUE_RX, &BACK_LEFT_QUEUE_TX, Some(back_left_boot0_pin), Some(back_left_reset_pin));
    let mut back_left_motor = WheelMotor::new(back_left_stm32_interface, wheel_firmware_image);


    /* BACK RIGHT WHEEL */
    // BackRight Wheel  - USART3 - tx pb10, rx pb11, boot pf4,  rst pa3

    // back right IO
    let back_right_uart_config = stm32_interface::get_bootloader_uart_config();
    let back_right_int = interrupt::take!(USART3);
    let back_right_usart = Uart::new(p.USART3, p.PB11, p.PB10, back_right_int, p.DMA1_CH6, p.DMA1_CH7, back_right_uart_config);
    let (back_right_tx, back_right_rx) = back_right_usart.split();
    let back_right_boot0_pin = Output::new(p.PF4, Level::Low, Speed::Medium); // boot0 not active

    // register back right uart primitives with executor
    spawner.spawn(BACK_RIGHT_QUEUE_RX.spawn_task(back_right_rx)).unwrap();
    spawner.spawn(BACK_RIGHT_QUEUE_TX.spawn_task(back_right_tx)).unwrap();

    // initialize the wheel
    let back_right_stm32_interface = Stm32Interface::new(&BACK_RIGHT_QUEUE_RX, &BACK_RIGHT_QUEUE_TX, Some(back_right_boot0_pin), Some(back_right_reset_pin));
    let mut back_right_motor = WheelMotor::new(back_right_stm32_interface, wheel_firmware_image);

    ////////////////////////////
    //  load firmware images  //
    ////////////////////////////

    // load firmware (use join?)
    // // defmt::info!("flashing front right");
    // front_right_motor.load_default_firmware_image().await;
    // // defmt::info!("flashing front left");
    // front_left_motor.load_default_firmware_image().await;
    // // defmt::info!("flashing back left");
    // back_left_motor.load_default_firmware_image().await;
    // // defmt::info!("flashing back right");
    // back_right_motor.load_default_firmware_image().await;
    let _res = embassy_futures::join::join4(front_right_motor.load_default_firmware_image(),
        front_left_motor.load_default_firmware_image(),
        back_left_motor.load_default_firmware_image(),
        back_right_motor.load_default_firmware_image()
    ).await;

    info!("after load firmware");

    // leave reset
    // don't pull the chip out of reset until we're ready to read packets or we'll fill the queue
    embassy_futures::join::join4(front_right_motor.leave_reset(), 
        front_left_motor.leave_reset(), 
        back_left_motor.leave_reset(), 
        back_right_motor.leave_reset()).await;

    info!("after leave reset");

    front_right_motor.set_telemetry_enabled(true);
    front_left_motor.set_telemetry_enabled(true);
    back_left_motor.set_telemetry_enabled(true);
    back_right_motor.set_telemetry_enabled(true);

    info!("after telemetry enable");

    // need to have telem off by default and enabled later
    // theres a race condition to begin processing packets from the first part out
    // of reset and waiting for the last part to boot up
    Timer::after(Duration::from_millis(5)).await;

    // front_right_motor.leave_reset().await;
    // front_left_motor.leave_reset().await;
    // back_left_motor.leave_reset().await;
    // back_right_motor.leave_reset().await;

    // robot params
    let robot_model_constants: RobotConstants = RobotConstants {
        wheel_angles_rad: Vector4::new(
            WHEEL_ANGLES_DEG[0].to_radians(),
            WHEEL_ANGLES_DEG[1].to_radians(),
            WHEEL_ANGLES_DEG[2].to_radians(),
            WHEEL_ANGLES_DEG[3].to_radians()),
        wheel_radius_m: Vector4::new(
            WHEEL_RADIUS_M,
            WHEEL_RADIUS_M,
            WHEEL_RADIUS_M,
            WHEEL_RADIUS_M),
        wheel_dist_to_cent_m: Vector4::new(
            WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
            WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
            WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
            WHEEL_DISTANCE_TO_ROBOT_CENTER_M),

    };

    let robot_model: RobotModel = RobotModel::new(robot_model_constants);

    info!("before");
    Timer::after(Duration::from_millis(1000)).await;

    /////////////////
    //  main loop  //
    /////////////////

    // 100 Hz loop rate
    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    // let mut angle: f32 = 0.0;
    let mut angle: f32 = core::f32::consts::PI / 2.0;
    loop {
        // info!("tick");
        front_right_motor.process_packets();
        front_left_motor.process_packets();
        back_left_motor.process_packets();
        back_right_motor.process_packets();

        let vel = 0.1; // DC
        let cmd_vel: Vector3<f32> = Vector3::new(libm::cosf(angle) * vel, libm::sinf(angle) * vel, 0.0);
        defmt::info!("cmd vel [{:?},{:?},{:?}]", cmd_vel[0], cmd_vel[1], cmd_vel[2]);
        let wheel_vels = robot_model.robot_vel_to_wheel_vel(cmd_vel);
        defmt::info!("set duty cycles [{:?},{:?},{:?},{:?}]", wheel_vels[0],  wheel_vels[1],  wheel_vels[2],  wheel_vels[3]);
        let rec_body_vel = robot_model.wheel_vel_to_robot_vel(wheel_vels);
        defmt::info!("rec_body_vel [{:?},{:?},{:?}]", rec_body_vel[0], rec_body_vel[1], rec_body_vel[2]);



        // let c_vel = libm::sinf(angle) / 2.0;
        // let c_vel = 0.2;
        front_right_motor.set_setpoint(wheel_vels[0]);
        front_left_motor.set_setpoint(wheel_vels[1]);
        back_left_motor.set_setpoint(wheel_vels[2]);
        back_right_motor.set_setpoint(wheel_vels[3]);
        angle += core::f32::consts::FRAC_2_PI / 200.0;

        // let c_vel = 0.2;
        // front_right_motor.set_setpoint(c_vel);
        // front_left_motor.set_setpoint(c_vel);
        // back_left_motor.set_setpoint(c_vel);
        // back_right_motor.set_setpoint(c_vel);
        // front_right_motor.set_setpoint(1.0);

        front_right_motor.send_motion_command();
        front_left_motor.send_motion_command();
        back_left_motor.send_motion_command();
        back_right_motor.send_motion_command();

        defmt::info!("FR = torque: {:?}, RPM {:?}", front_right_motor.read_current(), front_right_motor.read_rpm());
        // defmt::info!("encoder delta: {:?}", front_right_motor.read_encoder_delta());
        // defmt::info!("vel rad/s: {:?}", front_right_motor.read_rads() * 60.0 / (2.0 * PI));
        // defmt::info!("vel rad/s: {:?}", front_right_motor.read_rads());
        // defmt::info!("vel RPM: {:?}", front_right_motor.read_rads() * 60.0 / (2.0 * PI));



        main_loop_rate_ticker.next().await;
    }
}