use ateam_common_packets::bindings_radio::{BasicControl, BasicTelemetry};
use embassy_executor::SendSpawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::Uart;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::pubsub::Subscriber;
use embassy_time::{Duration, Timer};
use nalgebra::{Vector3, Vector4};

use crate::motion::robot_controller::BodyVelocityController;
use crate::motion::robot_model::{RobotConstants, RobotModel};
use crate::stm32_interface::Stm32Interface;
use crate::stspin_motor::{DribblerMotor, WheelMotor};
use crate::uart_queue::*;
use crate::{include_external_cpp_bin, pins::*, usart_buffer};
use crate::{queue, BATTERY_MIN_VOLTAGE};

include_external_cpp_bin! {DRIB_FW_IMG, "dribbler.bin"}
include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}

const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;
const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
usart_buffer!(
    FrontRight,
    MotorFRUart,
    MotorFRDmaRx,
    MotorFRDmaTx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH
);
usart_buffer!(
    FrontLeft,
    MotorFLUart,
    MotorFLDmaRx,
    MotorFLDmaTx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH
);
usart_buffer!(
    BackLeft,
    MotorBLUart,
    MotorBLDmaRx,
    MotorBLDmaTx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH
);
usart_buffer!(
    BackRight,
    MotorBRUart,
    MotorBRDmaRx,
    MotorBRDmaTx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH
);
usart_buffer!(
    Dribbler,
    MotorDUart,
    MotorDDmaRx,
    MotorDDmaTx,
    MAX_RX_PACKET_SIZE,
    RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE,
    TX_BUF_DEPTH
);

const TICKS_WITHOUT_PACKET_STOP: u16 = 25;

const WHEEL_ANGLES_DEG: Vector4<f32> = Vector4::new(30.0, 150.0, 225.0, 315.0);
const WHEEL_RADIUS_M: f32 = 0.049 / 2.0; // wheel dia 49mm
const WHEEL_DISTANCE_TO_ROBOT_CENTER_M: f32 = 0.085; // 85mm from center of wheel body to center of robot

pub struct Control<'a> {
    robot_model: RobotModel,
    robot_controller: BodyVelocityController<'a>,
    cmd_vel: Option<Vector3<f32>>,
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
        MotorFRBoot,
        MotorFRReset,
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
        MotorFLBoot,
        MotorFLReset,
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
        MotorBLBoot,
        MotorBLReset,
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
        MotorBRBoot,
        MotorBRReset,
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
        MotorDBoot,
        MotorDReset,
    >,
    ticks_since_packet: u16,
}

impl<'a> Control<'a> {
    pub fn new(
        spawner: &SendSpawner,
        front_right_usart: Uart<'static, MotorFRUart, MotorFRDmaTx, MotorFRDmaRx>,
        front_left_usart: Uart<'static, MotorFLUart, MotorFLDmaTx, MotorFLDmaRx>,
        back_left_usart: Uart<'static, MotorBLUart, MotorBLDmaTx, MotorBLDmaRx>,
        back_right_usart: Uart<'static, MotorBRUart, MotorBRDmaTx, MotorBRDmaRx>,
        drib_usart: Uart<'static, MotorDUart, MotorDDmaTx, MotorDDmaRx>,
        front_right_boot0: Output<'static, MotorFRBoot>,
        front_left_boot0: Output<'static, MotorFLBoot>,
        back_left_boot0: Output<'static, MotorBLBoot>,
        back_right_boot0: Output<'static, MotorBRBoot>,
        drib_boot0: Output<'static, MotorDBoot>,
        front_right_reset: Output<'static, MotorFRReset>,
        front_left_reset: Output<'static, MotorFLReset>,
        back_left_reset: Output<'static, MotorBLReset>,
        back_right_reset: Output<'static, MotorBRReset>,
        drib_reset: Output<'static, MotorDReset>,
        ball_detected_thresh: f32,
    ) -> Control<'a> {
        let wheel_firmware_image = WHEEL_FW_IMG;
        let drib_firmware_image = DRIB_FW_IMG;

        let (front_right_tx, front_right_rx) = front_right_usart.split();
        let (front_left_tx, front_left_rx) = front_left_usart.split();
        let (back_left_tx, back_left_rx) = back_left_usart.split();
        let (back_right_tx, back_right_rx) = back_right_usart.split();
        let (drib_tx, drib_rx) = drib_usart.split();

        spawner
            .spawn(FrontRight::QUEUE_RX.spawn_task(front_right_rx))
            .unwrap();
        spawner
            .spawn(FrontRight::QUEUE_TX.spawn_task(front_right_tx))
            .unwrap();
        spawner
            .spawn(FrontLeft::QUEUE_RX.spawn_task(front_left_rx))
            .unwrap();
        spawner
            .spawn(FrontLeft::QUEUE_TX.spawn_task(front_left_tx))
            .unwrap();
        spawner
            .spawn(BackLeft::QUEUE_RX.spawn_task(back_left_rx))
            .unwrap();
        spawner
            .spawn(BackLeft::QUEUE_TX.spawn_task(back_left_tx))
            .unwrap();
        spawner
            .spawn(BackRight::QUEUE_RX.spawn_task(back_right_rx))
            .unwrap();
        spawner
            .spawn(BackRight::QUEUE_TX.spawn_task(back_right_tx))
            .unwrap();
        spawner
            .spawn(Dribbler::QUEUE_RX.spawn_task(drib_rx))
            .unwrap();
        spawner
            .spawn(Dribbler::QUEUE_TX.spawn_task(drib_tx))
            .unwrap();

        let front_right_stm32_interface = Stm32Interface::new(
            &FrontRight::QUEUE_RX,
            &FrontRight::QUEUE_TX,
            Some(front_right_boot0),
            Some(front_right_reset),
        );
        let front_right_motor = WheelMotor::new(front_right_stm32_interface, wheel_firmware_image);

        let front_left_stm32_interface = Stm32Interface::new(
            &FrontLeft::QUEUE_RX,
            &FrontLeft::QUEUE_TX,
            Some(front_left_boot0),
            Some(front_left_reset),
        );
        let front_left_motor = WheelMotor::new(front_left_stm32_interface, wheel_firmware_image);

        let back_left_stm32_interface = Stm32Interface::new(
            &BackLeft::QUEUE_RX,
            &BackLeft::QUEUE_TX,
            Some(back_left_boot0),
            Some(back_left_reset),
        );
        let back_left_motor = WheelMotor::new(back_left_stm32_interface, wheel_firmware_image);

        let back_right_stm32_interface = Stm32Interface::new(
            &BackRight::QUEUE_RX,
            &BackRight::QUEUE_TX,
            Some(back_right_boot0),
            Some(back_right_reset),
        );
        let back_right_motor = WheelMotor::new(back_right_stm32_interface, wheel_firmware_image);

        let drib_stm32_interface = Stm32Interface::new(
            &Dribbler::QUEUE_RX,
            &Dribbler::QUEUE_TX,
            Some(drib_boot0),
            Some(drib_reset),
        );
        let drib_motor = DribblerMotor::new(
            drib_stm32_interface,
            drib_firmware_image,
            ball_detected_thresh,
        );

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

        let body_velocity_controller =
            BodyVelocityController::new_from_global_params(1.0 / 100.0, robot_model);

        Control {
            robot_model,
            robot_controller: body_velocity_controller,
            cmd_vel: None,
            drib_vel: 0.0,
            front_right_motor,
            front_left_motor,
            back_left_motor,
            back_right_motor,
            drib_motor,
            ticks_since_packet: 0,
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

    pub async fn tick(
        &mut self,
        latest_control: Option<BasicControl>,
        gyro_rads: f32,
        battery_v: f32,
    ) -> Option<BasicTelemetry> {
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
            self.cmd_vel = Some(cmd_vel);
            self.drib_vel = latest_control.dribbler_speed;
            self.ticks_since_packet = 0;
        } else {
            self.ticks_since_packet += 1;
            if self.ticks_since_packet >= TICKS_WITHOUT_PACKET_STOP {
                self.cmd_vel = None;
                self.ticks_since_packet = 0;
            }
        }

        let wheel_vels = if let Some(cmd_vel) = self.cmd_vel {
            // now we have setpoint r(t) in self.cmd_vel
            // let battery_v = self.battery_sub.next_message_pure().await as f32;
            let controls_enabled = true;
            // let gyro_rads =
            //     (self.gyro_sub.next_message_pure().await as f32) * 2.0 * core::f32::consts::PI / 360.0;
            if battery_v > BATTERY_MIN_VOLTAGE {
                if controls_enabled {
                    // TODO check order
                    let wheel_vels = Vector4::new(
                        self.front_right_motor.read_rads(),
                        self.front_left_motor.read_rads(),
                        self.back_left_motor.read_rads(),
                        self.back_right_motor.read_rads(),
                    );
    
                    // TODO read from channel or something
    
                    self.robot_controller
                        .control_update(&cmd_vel, &wheel_vels, gyro_rads);
    
                    self.robot_controller.get_wheel_velocities()
                } else {
                    self.robot_model.robot_vel_to_wheel_vel(cmd_vel)
                }
            } else {
                // Battery is too low, set velocity to zero
                Vector4::new(0.0, 0.0, 0.0, 0.0)
            }
        } else {
            Vector4::new(0.0, 0.0, 0.0, 0.0)
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
