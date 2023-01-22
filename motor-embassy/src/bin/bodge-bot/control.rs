use ateam_common_packets::bindings_radio::{BasicControl, BasicTelemetry};
use embassy_executor::SendSpawner;
use embassy_stm32::{
    gpio::{Level, Output, OutputOpenDrain, Pull, Speed},
    peripherals::{
        DMA1_CH0, DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, UART4,
        UART5, UART7, USART3,
    },
    usart::Uart,
};
use embassy_time::{Duration, Timer};
use motor_embassy::{
    include_external_cpp_bin,
    queue::Buffer,
    robot_model::{RobotConstants, RobotModel},
    stm32_interface::Stm32Interface,
    stspin_motor::WheelMotor,
    uart_queue::{UartReadQueue, UartWriteQueue},
};
use nalgebra::{Vector3, Vector4};

use crate::pins::{
    MotorBLBootPin, MotorBLDmaRx, MotorBLDmaTx, MotorBLResetPin, MotorBLUart,
    MotorBRBootPin, MotorBRDmaRx, MotorBRDmaTx, MotorBRResetPin, MotorBRUart,
    MotorFLBootPin, MotorFLDmaRx, MotorFLDmaTx, MotorFLResetPin, MotorFLUart,
    MotorFRBootPin, MotorFRDmaRx, MotorFRDmaTx, MotorFRResetPin, MotorFRUart,
};

include_external_cpp_bin! {STEVAL3204_DRIB_POTCTRL_FW_IMG, "dev3204-drib-potctrl.bin"}
include_external_cpp_bin! {STEVAL3204_DRIB_FW_IMG, "dev3204-drib.bin"}
include_external_cpp_bin! {STEVAL3204_WHEEL_FW_IMG, "dev3204-wheel.bin"}

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
static mut FRONT_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_TX: UartWriteQueue<UART5, DMA1_CH0, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_RX: UartReadQueue<UART5, DMA1_CH1, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_RX });

// buffers for front left
#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_TX: UartWriteQueue<UART7, DMA1_CH2, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_RX: UartReadQueue<UART7, DMA1_CH3, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_RX });

// buffers for back left
#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_LEFT_QUEUE_TX: UartWriteQueue<UART4, DMA1_CH4, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_LEFT_QUEUE_RX: UartReadQueue<UART4, DMA1_CH5, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_RX });

// buffers for back right
#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_TX: UartWriteQueue<USART3, DMA1_CH6, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_RX: UartReadQueue<USART3, DMA1_CH7, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_RX });

const WHEEL_ANGLES_DEG: Vector4<f32> = Vector4::new(30.0, 150.0, 225.0, 315.0);
const WHEEL_RADIUS_M: f32 = 0.049 / 2.0; // wheel dia 49mm
const WHEEL_DISTANCE_TO_ROBOT_CENTER_M: f32 = 0.085; // 85mm from center of wheel body to center of robot

pub struct Control {
    robot_model: RobotModel,
    cmd_vel: Vector3<f32>,
    front_right_motor: WheelMotor<
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
    front_left_motor: WheelMotor<
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
    back_left_motor: WheelMotor<
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
    back_right_motor: WheelMotor<
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
}

// Uart<UART5, DMA1_CH0, DMA1_CH1>

impl Control {
    pub fn new(
        spawner: &SendSpawner,
        front_right_usart: Uart<'static, MotorFRUart, MotorFRDmaTx, MotorFRDmaRx>,
        front_left_usart: Uart<'static, MotorFLUart, MotorFLDmaTx, MotorFLDmaRx>,
        back_left_usart: Uart<'static, MotorBLUart, MotorBLDmaTx, MotorBLDmaRx>,
        back_right_usart: Uart<'static, MotorBRUart, MotorBRDmaTx, MotorBRDmaRx>,
        front_right_boot0_pin: MotorFRBootPin,
        front_left_boot0_pin: MotorFLBootPin,
        back_left_boot0_pin: MotorBLBootPin,
        back_right_boot0_pin: MotorBRBootPin,
        front_right_reset_pin: MotorFRResetPin,
        front_left_reset_pin: MotorFLResetPin,
        back_left_reset_pin: MotorBLResetPin,
        back_right_reset_pin: MotorBRResetPin,
    ) -> Control {
        let wheel_firmware_image = STEVAL3204_WHEEL_FW_IMG;

        let (front_right_tx, front_right_rx) = front_right_usart.split();
        let (front_left_tx, front_left_rx) = front_left_usart.split();
        let (back_left_tx, back_left_rx) = back_left_usart.split();
        let (back_right_tx, back_right_rx) = back_right_usart.split();

        let front_right_boot0_pin = Output::new(front_right_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let front_left_boot0_pin = Output::new(front_left_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let back_left_boot0_pin = Output::new(back_left_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let back_right_boot0_pin = Output::new(back_right_boot0_pin, Level::Low, Speed::Medium); // boot0 not active

        let front_right_reset_pin =
            OutputOpenDrain::new(front_right_reset_pin, Level::Low, Speed::Medium, Pull::None); // reset active
        let front_left_reset_pin =
            OutputOpenDrain::new(front_left_reset_pin, Level::Low, Speed::Medium, Pull::None); // reset active
        let back_left_reset_pin =
            OutputOpenDrain::new(back_left_reset_pin, Level::Low, Speed::Medium, Pull::None); // reset active
        let back_right_reset_pin =
            OutputOpenDrain::new(back_right_reset_pin, Level::Low, Speed::Medium, Pull::None); // reset active

        spawner
            .spawn(FRONT_RIGHT_QUEUE_RX.spawn_task(front_right_rx))
            .unwrap();
        spawner
            .spawn(FRONT_RIGHT_QUEUE_TX.spawn_task(front_right_tx))
            .unwrap();
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

        let front_right_stm32_interface = Stm32Interface::new(
            &FRONT_RIGHT_QUEUE_RX,
            &FRONT_RIGHT_QUEUE_TX,
            Some(front_right_boot0_pin),
            Some(front_right_reset_pin),
        );
        let front_right_motor = WheelMotor::new(front_right_stm32_interface, wheel_firmware_image);

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
            front_right_motor,
            front_left_motor,
            back_left_motor,
            back_right_motor,
        }
    }

    pub async fn load_firmware(&mut self) {
        let _res = embassy_futures::join::join4(
            self.front_right_motor.load_default_firmware_image(),
            self.front_left_motor.load_default_firmware_image(),
            self.back_left_motor.load_default_firmware_image(),
            self.back_right_motor.load_default_firmware_image(),
        )
        .await;

        // leave reset
        // don't pull the chip out of reset until we're ready to read packets or we'll fill the queue
        embassy_futures::join::join4(
            self.front_right_motor.leave_reset(),
            self.front_left_motor.leave_reset(),
            self.back_left_motor.leave_reset(),
            self.back_right_motor.leave_reset(),
        )
        .await;

        self.front_right_motor.set_telemetry_enabled(true);
        self.front_left_motor.set_telemetry_enabled(true);
        self.back_left_motor.set_telemetry_enabled(true);
        self.back_right_motor.set_telemetry_enabled(true);

        // need to have telem off by default and enabled later
        // theres a race condition to begin processing packets from the first part out
        // of reset and waiting for the last part to boot up
        Timer::after(Duration::from_millis(10)).await;
    }

    pub fn tick(&mut self, latest_control: Option<BasicControl>) -> Option<BasicTelemetry> {
        self.front_right_motor.process_packets();
        self.front_left_motor.process_packets();
        self.back_left_motor.process_packets();
        self.back_right_motor.process_packets();

        // let vel = 0.0005; // DC
        // let angle: f32 = core::f32::consts::PI / 4.0;
        // let cmd_vel: Vector3<f32> =
        //     Vector3::new(libm::sinf(angle) * vel, libm::cosf(angle) * vel, 0.0);
        if let Some(latest_control) = &latest_control {
            let cmd_vel = Vector3::new(
                latest_control.vel_x_linear,
                latest_control.vel_y_linear,
                latest_control.vel_z_angular,
            );
            self.cmd_vel = cmd_vel;
        }
        let wheel_vels = self.robot_model.robot_vel_to_wheel_vel(self.cmd_vel);

        // let c_vel = libm::sinf(angle) / 2.0;
        // let c_vel = 0.2;
        self.front_right_motor.set_setpoint(wheel_vels[0]);
        self.front_left_motor.set_setpoint(wheel_vels[1]);
        self.back_left_motor.set_setpoint(wheel_vels[2]);
        self.back_right_motor.set_setpoint(wheel_vels[3]);
        // angle += core::f32::consts::FRAC_2_PI / 200.0;

        // let c_vel = 0.2;
        // front_right_motor.set_setpoint(c_vel);
        // front_left_motor.set_setpoint(c_vel);
        // back_left_motor.set_setpoint(c_vel);
        // back_right_motor.set_setpoint(c_vel);

        self.front_right_motor.send_motion_command();
        self.front_left_motor.send_motion_command();
        self.back_left_motor.send_motion_command();
        self.back_right_motor.send_motion_command();

        Some(BasicTelemetry {
            sequence_number: 0,
            robot_revision_major: 0,
            robot_revision_minor: 0,
            battery_level: 0.,
            battery_temperature: 0.,
            _bitfield_align_1: [],
            _bitfield_1: BasicTelemetry::new_bitfield_1(
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ),
            motor_0_temperature: 0.,
            motor_1_temperature: 0.,
            motor_2_temperature: 0.,
            motor_3_temperature: 0.,
            motor_4_temperature: 0.,
            kicker_charge_level: 0.,
        })
    }
}
