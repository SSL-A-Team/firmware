use ateam_common_packets::{bindings_radio::{BasicControl, BasicTelemetry, ControlDebugTelemetry, ParameterCommand, ParameterName}};
use embassy_executor::SendSpawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    usart::Uart,
};
use embassy_time::{Duration, Timer};
use ateam_control_board::{
    include_external_cpp_bin,
    queue::Buffer,
    stm32_interface::Stm32Interface,
    stspin_motor::{WheelMotor, DribblerMotor},
    uart_queue::{UartReadQueue, UartWriteQueue},
    motion::{
        robot_model::{RobotConstants, RobotModel},
        robot_controller::BodyVelocityController
    },
    BATTERY_MIN_VOLTAGE, parameter_interface::ParameterInterface, tasks::imu::{GyroSub, AccelSub}
};
use nalgebra::{Vector3, Vector4};

use embassy_sync::pubsub::Subscriber;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use crate::pins::*;


include_external_cpp_bin! {DRIB_FW_IMG, "dribbler.bin"}
include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}

// motor pinout
// FrontRight Wheel - UART5  - tx pb6,  pb12,    boot pb1,  rst pb2, DMA1 0/1
// FrontLeft Wheel  - UART7  - tx pf7,  rx pf6,  boot pg2,  rst pg3, DMA1 2/3
// BackLeft Wheel   - UART4  - tx pd1,  rx pd0,  boot pg0,  rst pg1, DMA1 4/5
// BackRight Wheel  - USART3 - tx pb10, rx pb11, boot pf4,  rst pa3, DMA1 6/7
// Dribbler         - USART6 - tx pc6,  rx pc7,  boot pc2,  rst pd7, DMA2 2/3

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

const TICKS_WITHOUT_PACKET_STOP: u16 = 25;

// buffers for front right
#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_TX: UartWriteQueue<MotorFRUart, MotorFRDmaTx, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_RIGHT_QUEUE_RX: UartReadQueue<MotorFRUart, MotorFRDmaRx, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_RIGHT_BUFFERS_RX });

// buffers for front left
#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_TX: UartWriteQueue<MotorFLUart, MotorFLDmaTx, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_RX: UartReadQueue<MotorFLUart, MotorFLDmaRx, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_RX });

// buffers for back left
#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_LEFT_QUEUE_TX: UartWriteQueue<MotorBLUart, MotorBLDmaTx, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_LEFT_QUEUE_RX: UartReadQueue<MotorBLUart, MotorBLDmaRx, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut BACK_LEFT_BUFFERS_RX });

// buffers for back right
#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_TX: UartWriteQueue<MotorBRUart, MotorBRDmaTx, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BACK_RIGHT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static BACK_RIGHT_QUEUE_RX: UartReadQueue<MotorBRUart, MotorBRDmaRx, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut BACK_RIGHT_BUFFERS_RX });

// buffers for dribbler
#[link_section = ".axisram.buffers"]
static mut DRIB_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static DRIB_QUEUE_TX: UartWriteQueue<MotorDUart, MotorDDmaTx, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut DRIB_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut DRIB_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static DRIB_QUEUE_RX: UartReadQueue<MotorDUart, MotorDDmaRx, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut DRIB_BUFFERS_RX });

const WHEEL_ANGLES_DEG: Vector4<f32> = Vector4::new(30.0, 150.0, 225.0, 315.0);
const WHEEL_RADIUS_M: f32 = 0.049 / 2.0; // wheel dia 49mm
const WHEEL_DISTANCE_TO_ROBOT_CENTER_M: f32 = 0.085; // 85mm from center of wheel body to center of robot

pub struct Control<'a> {
    robot_model: RobotModel,
    robot_controller: BodyVelocityController<'a>,
    cmd_vel: Vector3<f32>,
    drib_vel: f32,
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
    drib_motor: DribblerMotor<
        'static,
        MotorDUart,
        MotorDDmaRx,
        MotorDDmaTx,
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
        MotorDBootPin,
        MotorDResetPin,
    >,
    ticks_since_packet: u16,
    gyro_sub: GyroSub<'a>,
    accel_sub: AccelSub<'a>,
    last_gyro_vals: Vector3<f32>,
    last_accel_vals: Vector3<f32>,
    battery_sub: Subscriber<'static, ThreadModeRawMutex, f32, 2, 2, 2>
}

// Uart<UART5, DMA1_CH0, DMA1_CH1>

impl<'a> Control<'a> {
    pub fn new(
        spawner: &SendSpawner,
        front_right_usart: Uart<'static, MotorFRUart, MotorFRDmaTx, MotorFRDmaRx>,
        front_left_usart: Uart<'static, MotorFLUart, MotorFLDmaTx, MotorFLDmaRx>,
        back_left_usart: Uart<'static, MotorBLUart, MotorBLDmaTx, MotorBLDmaRx>,
        back_right_usart: Uart<'static, MotorBRUart, MotorBRDmaTx, MotorBRDmaRx>,
        drib_usart: Uart<'static, MotorDUart, MotorDDmaTx, MotorDDmaRx>,
        front_right_boot0_pin: MotorFRBootPin,
        front_left_boot0_pin: MotorFLBootPin,
        back_left_boot0_pin: MotorBLBootPin,
        back_right_boot0_pin: MotorBRBootPin,
        drib_boot0_pin: MotorDBootPin,
        front_right_reset_pin: MotorFRResetPin,
        front_left_reset_pin: MotorFLResetPin,
        back_left_reset_pin: MotorBLResetPin,
        back_right_reset_pin: MotorBRResetPin,
        drib_reset_pin: MotorDResetPin,
        ball_detected_thresh: f32,
        gyro_sub: GyroSub<'a>,
        accel_sub: AccelSub<'a>,
        battery_sub: Subscriber<'static, ThreadModeRawMutex, f32, 2, 2, 2>
    ) -> Control<'a> {
        let wheel_firmware_image = WHEEL_FW_IMG;
        let drib_firmware_image = DRIB_FW_IMG;

        let (front_right_tx, front_right_rx) = front_right_usart.split();
        let (front_left_tx, front_left_rx) = front_left_usart.split();
        let (back_left_tx, back_left_rx) = back_left_usart.split();
        let (back_right_tx, back_right_rx) = back_right_usart.split();
        let (drib_tx, drib_rx) = drib_usart.split();

        let front_right_boot0_pin = Output::new(front_right_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let front_left_boot0_pin = Output::new(front_left_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let back_left_boot0_pin = Output::new(back_left_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let back_right_boot0_pin = Output::new(back_right_boot0_pin, Level::Low, Speed::Medium); // boot0 not active
        let drib_boot0_pin = Output::new(drib_boot0_pin, Level::Low, Speed::Medium); // boot0 not active

        let front_right_reset_pin =
            Output::new(front_right_reset_pin, Level::Low, Speed::Medium); // reset active
        let front_left_reset_pin =
            Output::new(front_left_reset_pin, Level::Low, Speed::Medium); // reset active
        let back_left_reset_pin =
            Output::new(back_left_reset_pin, Level::Low, Speed::Medium); // reset active
        let back_right_reset_pin =
            Output::new(back_right_reset_pin, Level::Low, Speed::Medium); // reset active
        let drib_reset_pin =
            Output::new(drib_reset_pin, Level::Low, Speed::Medium); // reset active

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
        spawner                    // // // info!("{:?}", defmt::Debug2Format(&mrp.data.motion));
        // // info!("\n");
        // // // info!("vel set {:?}", mrp.data.motion.vel_setpoint + 0.);
        // info!("vel enc {:?}", mrp.data.motion.vel_enc_estimate + 0.);
        // // // info!("vel hall {:?}", mrp.data.motion.vel_hall_estimate +                    // // // info!("{:?}", defmt::Debug2Format(&mrp.data.motion));
                    // // info!("\n");
                    // // // info!("vel set {:?}", mrp.data.motion.vel_setpoint + 0.);
                    // info!("vel enc {:?}", mrp.data.motion.vel_enc_estimate + 0.);
                    // // // info!("vel hall {:?}", mrp.data.motion.vel_hall_estimate + 0.);
                    // info!("master_error {:?}", mrp.data.motion.master_error());
                    // info!("{:?}", &mrp.data.motion._bitfield_1.get(0, 32));
                    // info!("hall_power_error {:?}", mrp.data.motion.hall_power_error());
                    // info!("hall_disconnected_error {:?}", mrp.data.motion.hall_disconnected_error());
                    // info!("bldc_transition_error {:?}", mrp.data.motion.bldc_transition_error());
                    // info!("bldc_commutation_watchdog_error {:?}", mrp.data.motion.bldc_commutation_watchdog_error());
                    // info!("enc_disconnected_error {:?}", mrp.data.motion.enc_disconnected_error());
                    // info!("enc_decoding_error {:?}", mrp.data.motion.enc_decoding_error());
                    // info!("hall_enc_vel_disagreement_error {:?}", mrp.data.motion.hall_enc_vel_disagreement_error());
                    // info!("overcurrent_error {:?}", mrp.data.motion.overcurrent_error());
                    // info!("undervoltage_error {:?}", mrp.data.motion.undervoltage_error());
                    // info!("overvoltage_error {:?}", mrp.data.motion.overvoltage_error());
                    // info!("torque_limited {:?}", mrp.data.motion.torque_limited());
                    // info!("control_loop_time_error {:?}", mrp.data.motion.control_loop_time_error());
                    // info!("reset_watchdog_independent {:?}", mrp.data.motion.reset_watchdog_independent());
                    // info!("reset_watchdog_window {:?}", mrp.data.motion.reset_watchdog_window());
                    // info!("reset_low_power {:?}", mrp.data.motion.reset_low_power());
                    // info!("reset_software {:?}", mrp.data.motion.reset_software()); 0.);
        // info!("master_error {:?}", mrp.data.motion.master_error());
        // info!("{:?}", &mrp.data.motion._bitfield_1.get(0, 32));
        // info!("hall_power_error {:?}", mrp.data.motion.hall_power_error());
        // info!("hall_disconnected_error {:?}", mrp.data.motion.hall_disconnected_error());
        // info!("bldc_transition_error {:?}", mrp.data.motion.bldc_transition_error());
        // info!("bldc_commutation_watchdog_error {:?}", mrp.data.motion.bldc_commutation_watchdog_error());
        // info!("enc_disconnected_error {:?}", mrp.data.motion.enc_disconnected_error());
        // info!("enc_decoding_error {:?}", mrp.data.motion.enc_decoding_error());
        // info!("hall_enc_vel_disagreement_error {:?}", mrp.data.motion.hall_enc_vel_disagreement_error());
        // info!("overcurrent_error {:?}", mrp.data.motion.overcurrent_error());
        // info!("undervoltage_error {:?}", mrp.data.motion.undervoltage_error());
        // info!("overvoltage_error {:?}", mrp.data.motion.overvoltage_error());
        // info!("torque_limited {:?}", mrp.data.motion.torque_limited());
        // info!("control_loop_time_error {:?}", mrp.data.motion.control_loop_time_error());
        // info!("reset_watchdog_independent {:?}", mrp.data.motion.reset_watchdog_independent());
        // info!("reset_watchdog_window {:?}", mrp.data.motion.reset_watchdog_window());
        // info!("reset_low_power {:?}", mrp.data.motion.reset_low_power());
        // info!("reset_software {:?}", mrp.data.motion.reset_software());
            .spawn(BACK_LEFT_QUEUE_TX.spawn_task(back_left_tx))
            .unwrap();
        spawner
            .spawn(BACK_RIGHT_QUEUE_RX.spawn_task(back_right_rx))
            .unwrap();
        spawner
            .spawn(BACK_RIGHT_QUEUE_TX.spawn_task(back_right_tx))
            .unwrap();
        spawner
            .spawn(DRIB_QUEUE_RX.spawn_task(drib_rx))
            .unwrap();
        spawner
            .spawn(DRIB_QUEUE_TX.spawn_task(drib_tx))
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

        let drib_stm32_interface = Stm32Interface::new(
            &DRIB_QUEUE_RX,
            &DRIB_QUEUE_TX,
            Some(drib_boot0_pin),
            Some(drib_reset_pin),
        );
        let drib_motor = DribblerMotor::new(drib_stm32_interface, drib_firmware_image, ball_detected_thresh);

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

        let body_velocity_controller = BodyVelocityController::new_from_global_params(1.0 / 100.0, robot_model);

        Control {
            robot_model,
            robot_controller: body_velocity_controller,
            cmd_vel: Vector3::new(0., 0., 0.),
            drib_vel: 0.0,
            front_right_motor,
            front_left_motor,
            back_left_motor,
            back_right_motor,
            drib_motor,
            ticks_since_packet: 0,
            gyro_sub,
            accel_sub,
            last_gyro_vals: Vector3::new(0., 0., 0.),
            last_accel_vals: Vector3::new(0., 0., 0.),
            battery_sub
        }
    }

    pub async fn load_firmware(&mut self) {
        let _res = embassy_futures::join::join5(
            self.front_right_motor.load_default_firmware_image(),
            self.front_left_motor.load_default_firmware_image(),
            self.back_left_motor.load_default_firmware_image(),
            self.back_right_motor.load_default_firmware_image(),
            self.drib_motor.load_default_firmware_image(),
        )
        .await;

        // defmt::info!("flashing firmware");

        // self.front_right_motor.load_default_firmware_image().await;
        // defmt::info!("FR flashed");

        // self.front_left_motor.load_default_firmware_image().await;
        // defmt::info!("FL flashed");

        // self.back_left_motor.load_default_firmware_image().await;
        // defmt::info!("BL flashed");

        // self.back_right_motor.load_default_firmware_image().await;
        // defmt::info!("BR flashed");

        // self.drib_motor.load_default_firmware_image().await;
        // defmt::info!("DRIB flashed");



        defmt::info!("flashed");

        // leave reset
        // don't pull the chip out of reset until we're ready to read packets or we'll fill the queue
        embassy_futures::join::join5(
            self.front_right_motor.leave_reset(),
            self.front_left_motor.leave_reset(),
            self.back_left_motor.leave_reset(),
            self.back_right_motor.leave_reset(),
            self.drib_motor.leave_reset(),
        )
        .await;

        self.front_right_motor.set_telemetry_enabled(true);
        self.front_left_motor.set_telemetry_enabled(true);
        self.back_left_motor.set_telemetry_enabled(true);
        self.back_right_motor.set_telemetry_enabled(true);
        self.drib_motor.set_telemetry_enabled(true);

        // need to have telem off by default and enabled later
        // theres a race condition to begin processing packets from the first part out
        // of reset and waiting for the last part to boot up
        Timer::after(Duration::from_millis(10)).await;
    }

    fn process_mc_packets(&mut self) {
        self.front_right_motor.process_packets();
        self.front_left_motor.process_packets();
        self.back_left_motor.process_packets();
        self.back_right_motor.process_packets();
        self.drib_motor.process_packets();
    }

    pub async fn tick(&mut self, latest_control: Option<BasicControl>) -> (Option<BasicTelemetry>, ControlDebugTelemetry) {
        self.process_mc_packets();

        self.front_right_motor.log_reset("FR");
        self.front_left_motor.log_reset("RL");
        self.back_left_motor.log_reset("BL");
        self.back_right_motor.log_reset("BR");
        self.drib_motor.log_reset("DRIB");

        if self.drib_motor.ball_detected() {
            defmt::info!("ball detected");
        }

        if let Some(latest_control) = &latest_control {
            let cmd_vel = Vector3::new(
                latest_control.vel_x_linear,
                latest_control.vel_y_linear,
                latest_control.vel_z_angular,
            );
            self.cmd_vel = cmd_vel;
            self.drib_vel = latest_control.dribbler_speed;
            self.ticks_since_packet = 0;
        } else {
            self.ticks_since_packet += 1;
            if self.ticks_since_packet >= TICKS_WITHOUT_PACKET_STOP {
                self.cmd_vel = Vector3::new(0., 0., 0.);
                self.ticks_since_packet = 0;
            }
        }

        // update with any new gyro vals
        if let Some(gyro_vals) = self.gyro_sub.try_next_message_pure() {
            self.last_gyro_vals = gyro_vals;
        }

        // update with any new accel vals
        if let Some(accel_vals) = self.accel_sub.try_next_message_pure() {
            self.last_accel_vals = accel_vals;
        }

        // now we have setpoint r(t) in self.cmd_vel
        let battery_v = self.battery_sub.next_message_pure().await as f32;
        let controls_enabled = true;

        let wheel_vels = if battery_v > BATTERY_MIN_VOLTAGE {
            if controls_enabled 
            {
                // TODO check order
                let wheel_vels = Vector4::new(
                    self.front_right_motor.read_rads(),
                    self.front_left_motor.read_rads(),
                    self.back_left_motor.read_rads(),
                    self.back_right_motor.read_rads()
                );

                // torque values are computed on the spin but put in the current variable
                // TODO update this when packet/var names are updated to match software
                let wheel_torques = Vector4::new(
                    self.front_right_motor.read_current(),
                    self.front_left_motor.read_current(),
                    self.back_left_motor.read_current(),
                    self.back_right_motor.read_current()
                );
            
                // TODO read from channel or something

                self.robot_controller.control_update(&self.cmd_vel, &wheel_vels, &wheel_torques, self.last_gyro_vals[2]);
            
                self.robot_controller.get_wheel_velocities()
            } 
            else 
            {
                self.robot_model.robot_vel_to_wheel_vel(self.cmd_vel)
            }
        }
        else
        {
            // Battery is too low, set velocity to zero
            Vector4::new(
                0.0,
                0.0,
                0.0,
                0.0)
        };

        self.front_right_motor.set_setpoint(wheel_vels[0]);
        self.front_left_motor.set_setpoint(wheel_vels[1]);
        self.back_left_motor.set_setpoint(wheel_vels[2]);
        self.back_right_motor.set_setpoint(wheel_vels[3]);

        let drib_dc = -1.0 * self.drib_vel / 1000.0;
        self.drib_motor.set_setpoint(drib_dc);

        self.front_right_motor.send_motion_command();
        self.front_left_motor.send_motion_command();
        self.back_left_motor.send_motion_command();
        self.back_right_motor.send_motion_command();
        self.drib_motor.send_motion_command();

        (Some(BasicTelemetry {
            sequence_number: 0,
            robot_revision_major: 0,
            robot_revision_minor: 0,
            battery_level: battery_v,
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
        }),
        self.robot_controller.get_control_debug_telem())
    }


}

impl<'a> ParameterInterface for Control<'a> {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool {
        return self.robot_controller.processes_cmd(param_cmd);
    }

    fn has_name(&self, param_name: ParameterName::Type) -> bool {
        return self.robot_controller.has_name(param_name);
    }

    fn apply_command(&mut self, param_cmd: &ParameterCommand) -> Result<ParameterCommand, ParameterCommand> {
        return self.robot_controller.apply_command(param_cmd);
    }
}
