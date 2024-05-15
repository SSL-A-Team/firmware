use ateam_control_board::{
    include_external_cpp_bin,
    queue::Buffer,
    motion::robot_model::{RobotConstants, RobotModel},
    motion::params::robot_physical_params::{
        WHEEL_ANGLES_DEG,
        WHEEL_RADIUS_M,
        WHEEL_DISTANCE_TO_ROBOT_CENTER_M
    },
    stm32_interface::Stm32Interface,
    stspin_motor::{DribblerMotor, WheelMotor},
    uart_queue::{UartReadQueue, UartWriteQueue},
};
use embassy_executor::SendSpawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    usart::Uart,
};
use embassy_time::{Duration, Timer};
use nalgebra::{Vector3, Vector4};

use crate::pins::*;

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}

// motor pinout
// FrontLeft Wheel  - UART7  - tx pf7,  rx pf6,  boot pg2,  rst pg3, DMA1 2/3
// BackLeft Wheel   - UART4  - tx pd1,  rx pd0,  boot pg0,  rst pg1, DMA1 4/5
// BackRight Wheel  - USART3 - tx pb10, rx pb11, boot pf4,  rst pa3, DMA1 6/7
// FrontRight Wheel - UART5  - tx pb6,  pb12,    boot pb1,  rst pb2, DMA1 0/1

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

// buffers for front left
#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_TX: UartWriteQueue<
    MotorFLUart,
    MotorFLDmaTx,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH,
> = UartWriteQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_RX: UartReadQueue<
    MotorFLUart,
    MotorFLDmaRx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
> = UartReadQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_RX });

// buffers for back left
#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_LEFT_QUEUE_TX: UartWriteQueue<
    MotorBLUart,
    MotorBLDmaTx,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH,
> = UartWriteQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_LEFT_QUEUE_RX: UartReadQueue<
    MotorBLUart,
    MotorBLDmaRx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
> = UartReadQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_RX });

// buffers for back right
#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_TX: UartWriteQueue<
    MotorBRUart,
    MotorBRDmaTx,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH,
> = UartWriteQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_RX: UartReadQueue<
    MotorBRUart,
    MotorBRDmaRx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
> = UartReadQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_RX });

// buffers for front right
#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_TX: UartWriteQueue<
    MotorFRUart,
    MotorFRDmaTx,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH,
> = UartWriteQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_RX: UartReadQueue<
    MotorFRUart,
    MotorFRDmaRx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
> = UartReadQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_RX });

pub struct MotorsError<T> {
    pub front_left: T,
    pub back_right: T,
    pub back_left: T,
    pub front_right: T
}

pub struct Control {
    robot_model: RobotModel,
    cmd_vel: Vector3<f32>,
    pub front_left_motor: WheelMotor<
        'static,
        MotorFLUart,
        MotorFLDmaRx,
        MotorFLDmaTx,
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
        MotorFLBootPin,
        MotorFLResetPin,
    >,
    pub back_left_motor: WheelMotor<
        'static,
        MotorBLUart,
        MotorBLDmaRx,
        MotorBLDmaTx,
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
        MotorBLBootPin,
        MotorBLResetPin,
    >,
    pub back_right_motor: WheelMotor<
        'static,
        MotorBRUart,
        MotorBRDmaRx,
        MotorBRDmaTx,
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
        MotorBRBootPin,
        MotorBRResetPin,
    >,
    pub front_right_motor: WheelMotor<
        'static,
        MotorFRUart,
        MotorFRDmaRx,
        MotorFRDmaTx,
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
        MotorFRBootPin,
        MotorFRResetPin,
    >,
}

impl Control {
    pub fn new(
        spawner: &SendSpawner,
        front_left_usart: Uart<'static, MotorFLUart, MotorFLDmaTx, MotorFLDmaRx>,
        back_left_usart: Uart<'static, MotorBLUart, MotorBLDmaTx, MotorBLDmaRx>,
        back_right_usart: Uart<'static, MotorBRUart, MotorBRDmaTx, MotorBRDmaRx>,
        front_right_usart: Uart<'static, MotorFRUart, MotorFRDmaTx, MotorFRDmaRx>,
        front_left_boot0_pin: MotorFLBootPin,
        back_left_boot0_pin: MotorBLBootPin,
        back_right_boot0_pin: MotorBRBootPin,
        front_right_boot0_pin: MotorFRBootPin,
        front_left_reset_pin: MotorFLResetPin,
        back_left_reset_pin: MotorBLResetPin,
        back_right_reset_pin: MotorBRResetPin,
        front_right_reset_pin: MotorFRResetPin,
    ) -> Control {
        let wheel_firmware_image = WHEEL_FW_IMG;

        let (front_left_tx, front_left_rx) = front_left_usart.split();
        let (back_left_tx, back_left_rx) = back_left_usart.split();
        let (back_right_tx, back_right_rx) = back_right_usart.split();
        let (front_right_tx, front_right_rx) = front_right_usart.split();

        let front_left_boot0_pin = Output::new(front_left_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let back_left_boot0_pin = Output::new(back_left_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let back_right_boot0_pin = Output::new(back_right_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let front_right_boot0_pin = Output::new(front_right_boot0_pin, Level::Low, Speed::Medium); // boot0 not active

        let front_left_reset_pin = Output::new(front_left_reset_pin, Level::Low, Speed::Medium); // reset active
        let back_left_reset_pin = Output::new(back_left_reset_pin, Level::Low, Speed::Medium); // reset active
        let back_right_reset_pin = Output::new(back_right_reset_pin, Level::Low, Speed::Medium); // reset active
        let front_right_reset_pin = Output::new(front_right_reset_pin, Level::Low, Speed::Medium); // reset active

        spawner
            .spawn(FRONT_LEFT_QUEUE_RX.spawn_task(front_left_rx))
            .unwrap();
        spawner
            .spawn(FRONT_LEFT_QUEUE_TX.spawn_task(front_left_tx))
            .unwrap();
        spawner
            .spawn(BACK_LEFT_QUEUE_RX.spawn_task(back_left_rx))
            .unwrap();
        spawner
            .spawn(BACK_LEFT_QUEUE_TX.spawn_task(back_left_tx))
            .unwrap();
        spawner
            .spawn(BACK_RIGHT_QUEUE_RX.spawn_task(back_right_rx))
            .unwrap();
        spawner
            .spawn(BACK_RIGHT_QUEUE_TX.spawn_task(back_right_tx))
            .unwrap();
        spawner
            .spawn(FRONT_RIGHT_QUEUE_RX.spawn_task(front_right_rx))
            .unwrap();
        spawner
            .spawn(FRONT_RIGHT_QUEUE_TX.spawn_task(front_right_tx))
            .unwrap();

        let front_left_stm32_interface = Stm32Interface::new(
            &FRONT_LEFT_QUEUE_RX,
            &FRONT_LEFT_QUEUE_TX,
            Some(front_left_boot0_pin),
            Some(front_left_reset_pin),
        );
        let front_left_motor = WheelMotor::new(front_left_stm32_interface, wheel_firmware_image);

        let back_left_stm32_interface = Stm32Interface::new(
            &BACK_LEFT_QUEUE_RX,
            &BACK_LEFT_QUEUE_TX,
            Some(back_left_boot0_pin),
            Some(back_left_reset_pin),
        );
        let back_left_motor = WheelMotor::new(back_left_stm32_interface, wheel_firmware_image);

        let back_right_stm32_interface = Stm32Interface::new(
            &BACK_RIGHT_QUEUE_RX,
            &BACK_RIGHT_QUEUE_TX,
            Some(back_right_boot0_pin),
            Some(back_right_reset_pin),
        );
        let back_right_motor = WheelMotor::new(back_right_stm32_interface, wheel_firmware_image);

        let front_right_stm32_interface = Stm32Interface::new(
            &FRONT_RIGHT_QUEUE_RX,
            &FRONT_RIGHT_QUEUE_TX,
            Some(front_right_boot0_pin),
            Some(front_right_reset_pin),
        );
        let front_right_motor = WheelMotor::new(front_right_stm32_interface, wheel_firmware_image);

        let robot_model_constants: RobotConstants = RobotConstants {
            wheel_angles_rad: Vector4::new(
                WHEEL_ANGLES_DEG[0].to_radians(),
                WHEEL_ANGLES_DEG[1].to_radians(),
                WHEEL_ANGLES_DEG[2].to_radians(),
                WHEEL_ANGLES_DEG[3].to_radians(),
            ),
            wheel_radius_m: Vector4::new(
                WHEEL_RADIUS_M,
                WHEEL_RADIUS_M,
                WHEEL_RADIUS_M,
                WHEEL_RADIUS_M,
            ),
            wheel_dist_to_cent_m: Vector4::new(
                WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
            ),
        };

        let robot_model: RobotModel = RobotModel::new(robot_model_constants);

        Control {
            robot_model,
            cmd_vel: Vector3::new(0., 0., 0.),
            front_left_motor,
            back_left_motor,
            back_right_motor,
            front_right_motor
        }
    }

    pub async fn load_firmware(&mut self) -> Result<(), MotorsError<bool>> {
        let ret = embassy_futures::join::join4(
            self.front_left_motor.load_default_firmware_image(),
            self.back_left_motor.load_default_firmware_image(),
            self.back_right_motor.load_default_firmware_image(),
            self.front_right_motor.load_default_firmware_image(),
        )
        .await;

        if ret.0.is_err() || ret.1.is_err() || ret.2.is_err() || ret.3.is_err() {
            return Err(MotorsError {
                front_left: ret.0.is_err(),
                back_left: ret.1.is_err(),
                back_right: ret.2.is_err(),
                front_right: ret.3.is_err()
            });
        }

        embassy_futures::join::join4(
            self.front_left_motor.leave_reset(),
            self.back_left_motor.leave_reset(),
            self.back_right_motor.leave_reset(),
            self.front_right_motor.leave_reset()
            )
        .await;

        self.front_left_motor.set_telemetry_enabled(true);
        self.back_left_motor.set_telemetry_enabled(true);
        self.back_right_motor.set_telemetry_enabled(true);
        self.front_right_motor.set_telemetry_enabled(true);

        Timer::after(Duration::from_millis(10)).await;

        Ok(())
    }

    pub fn tick(&mut self, vel_angular: f32) {
        self.front_left_motor.process_packets();
        self.back_left_motor.process_packets();
        self.back_right_motor.process_packets();
        self.front_right_motor.process_packets();

        self.front_left_motor.log_reset("FL");
        self.back_left_motor.log_reset("BL");
        self.back_right_motor.log_reset("BR");
        self.front_right_motor.log_reset("FR");

        let cmd_vel = Vector3::new(0., 0., vel_angular);
        self.cmd_vel = cmd_vel;
        let wheel_vels = self.robot_model.robot_vel_to_wheel_vel(&self.cmd_vel);

        self.front_left_motor.set_setpoint(wheel_vels[0]);
        self.back_left_motor.set_setpoint(wheel_vels[1]);
        self.back_right_motor.set_setpoint(wheel_vels[2]);
        self.front_right_motor.set_setpoint(wheel_vels[3]);

        self.front_left_motor.send_motion_command();
        self.back_left_motor.send_motion_command();
        self.back_right_motor.send_motion_command();
        self.front_right_motor.send_motion_command();
    }
}
