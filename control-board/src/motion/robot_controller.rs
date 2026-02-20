use core::f32::consts::PI;

use crate::parameter_interface::ParameterInterface;
use crate::motion::pid::PidController;
use ateam_common_packets::bindings::{
    ParameterCommandCode::*, ParameterDataFormat, ParameterName
};
use ateam_controls::{Vector3f, Vector4f, Vector6f, Vector8f};
use ateam_controls::robot_model::RobotModel;
use embassy_time::{Duration, Instant};
use nalgebra::{vector, SVector};

use ateam_common_packets::bindings::{ExtendedTelemetry, ParameterCommand};

// TODO find some general numeric type trait(s) for D
// Clamp the vector, but keep the direction consistent.
pub fn clamp_vector_keep_dir<const D: usize>(
    vec: &SVector<f32, D>,
    limits_abs: &SVector<f32, D>,
) -> SVector<f32, D> {
    // Applies the clamping in a sign-less way by getting the scale of the vector compared to absolute value clamp region.
    let scales: SVector<f32, D> = vec.abs().component_div(limits_abs);
    // To maintain direction, do a scalar divison of the max of the scales. Limit to make sure to only scale down.
    vec / f32::max(1.0, scales.max())
}


pub struct BodyController {
    pub robot_model: RobotModel,
    pub pid_controller: PidController<3>,
    pub body_twist_cmd: Vector3f,
    pub body_accel_cmd: Vector3f,
    pub wheel_vel_cmd: Vector4f,
    pub wheel_torque_cmd: Vector4f,
    pub debug_telemetry: ExtendedTelemetry,
    pub loop_period: Duration,
}

impl BodyController {
    pub fn new(loop_period: Duration) -> BodyController {
        // let pid_gains = nalgebra::matrix![
        //     // P,    I,    D,    I_min, I_max
        //      2.0,  0.5,  0.0, -0.1,   0.1;   // X
        //      2.0,  0.5,  0.0, -0.1,   0.1;   // Y
        //      3.0,  0.5,  0.0, -PI/2., PI/2.;   // Theta
        // ];
        let pid_gains = nalgebra::matrix![
            // P,    I,    D,    I_min, I_max
             1.5,  0.5,  0.0, -0.2,   0.1;   // X
             1.5,  0.5,  0.0, -0.2,   0.1;   // Y
             1.0,  0.5,  0.0, -1.0,   1.0;   // Theta
        ];
        BodyController {
            robot_model: RobotModel::new_from_default_params(loop_period.as_micros() as f32 * 1e-6),
            pid_controller: PidController::from_gains_matrix(&pid_gains),
            body_twist_cmd: Vector3f::default(),
            body_accel_cmd: Vector3f::default(),
            wheel_vel_cmd: Vector4f::default(),
            wheel_torque_cmd: Vector4f::default(),
            debug_telemetry: Default::default(),
            loop_period,
        }
    }

    pub fn control_update(
        &mut self,
        body_cmd: Vector3f,
        body_pose_control_enabled: bool,
        body_twist_control_enabled: bool,
        body_accel_control_enabled: bool,
        vision_pose_meas: Vector3f,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        gyro_theta_meas: f32,
        trace: bool,
    ) {
        let mut start = Instant::now();

        let state_prediction = self.robot_model.x;

        let measurement: Vector8f = vector![
            vision_pose_meas.x,
            vision_pose_meas.y,
            vision_pose_meas.z,
            wheel_vel_meas.x,
            wheel_vel_meas.y,
            wheel_vel_meas.z,
            wheel_vel_meas.w,
            gyro_theta_meas,
        ];
        self.robot_model.kf_update(measurement, !vision_update, false, false);

        let kf_update_time = Instant::now() - start;
        start = Instant::now();

        let mut state_estimate = self.robot_model.x;
        ///////////////// REMOVE AFTER TESTING ////////////////////////
        // state_estimate.z = 0.0;
        //////////////////////////////////////////////////////////////
        if trace {
            defmt::trace!("State Estimate: [{}, {}, {}, {}, {}, {}]",
                state_estimate.x,
                state_estimate.y,
                state_estimate.z,
                state_estimate.w,
                state_estimate.a,
                state_estimate.b,
            );
        }
        // defmt::info!("State estimate pose: {}, {}, {}", state_estimate.x, state_estimate.y, state_estimate.z);
        // defmt::info!("State estimate twist: {}, {}, {}", state_estimate.w, state_estimate.a, state_estimate.b);

        if body_pose_control_enabled {
            self.compute_effort_pose_control(state_estimate, body_cmd);
        } else if body_twist_control_enabled {
            self.compute_effort_twist_control(state_estimate, body_cmd);
        } else if body_accel_control_enabled {
            self.compute_effort_accel_control(state_estimate, body_cmd);
        } else {
            self.body_twist_cmd = Vector3f::default();
            self.body_accel_cmd = Vector3f::default();
            self.wheel_vel_cmd = Vector4f::default();
            self.wheel_torque_cmd = Vector4f::default();
        }

        let effort_time = Instant::now() - start;
        start = Instant::now();

        // defmt::info!("Wheel Velocity Cmds: {}, {}, {}, {}", self.wheel_vel_cmd.x, self.wheel_vel_cmd.y, self.wheel_vel_cmd.z, self.wheel_vel_cmd.w);

        // Safety checks on commands
        let max_wheel_vel = 30.0; // rad/s
        if self.wheel_vel_cmd.x.abs() > max_wheel_vel || self.wheel_vel_cmd.y.abs() > max_wheel_vel || self.wheel_vel_cmd.z.abs() > max_wheel_vel || self.wheel_vel_cmd.w.abs() > max_wheel_vel {
            // defmt::warn!("Wheel vel cmd too high: {}, {}, {}, {}", self.wheel_vel_cmd.x, self.wheel_vel_cmd.y, self.wheel_vel_cmd.z, self.wheel_vel_cmd.w);
            // self.wheel_vel_cmd = Vector4f::default();
            // self.wheel_torque_cmd = Vector4f::default();
        }
        if self.wheel_torque_cmd.x > 1.0 || self.wheel_torque_cmd.y > 1.0 || self.wheel_torque_cmd.z > 1.0 || self.wheel_torque_cmd.w > 1.0 {
            defmt::warn!("Wheel torque cmd too high: {}, {}, {}, {}", self.wheel_torque_cmd.x, self.wheel_torque_cmd.y, self.wheel_torque_cmd.z, self.wheel_torque_cmd.w);
            self.wheel_vel_cmd = Vector4f::default();
            self.wheel_torque_cmd = Vector4f::default();
        }

        // Copy values to telemetry
        self.debug_telemetry.set_body_pose_control_enabled(body_pose_control_enabled as u32);
        self.debug_telemetry.set_body_twist_control_enabled(body_twist_control_enabled as u32);
        self.debug_telemetry.set_body_accel_control_enabled(body_accel_control_enabled as u32);
        self.debug_telemetry.set_vision_update(vision_update as u32);
        self.debug_telemetry.imu_gyro[2] = gyro_theta_meas;
        self.debug_telemetry.vision_pose.copy_from_slice(&vision_pose_meas.as_slice());
        self.debug_telemetry.body_cmd.copy_from_slice(body_cmd.as_slice());
        self.debug_telemetry.kf_body_pose_prediction.copy_from_slice(&state_prediction.as_slice()[0..3]);
        self.debug_telemetry.kf_body_twist_prediction.copy_from_slice(&state_prediction.as_slice()[3..6]);
        self.debug_telemetry.kf_body_pose_estimate.copy_from_slice(&state_estimate.as_slice()[0..3]);
        self.debug_telemetry.kf_body_twist_estimate.copy_from_slice(&state_estimate.as_slice()[3..6]);
        self.debug_telemetry.body_twist_u.copy_from_slice(self.body_twist_cmd.as_slice());
        self.debug_telemetry.body_accel_u.copy_from_slice(self.body_accel_cmd.as_slice());

        let control_outputs_time = Instant::now() - start;
        start = Instant::now();

        // TODO: kalman filter predict next state
        // // Use control law adjusted value to predict the next cycle's state.
        // self.body_vel_filter.predict(&wheel_vel_output);

        self.robot_model.kf_predict(self.body_accel_cmd);

        let kf_predict_time = Instant::now() - start;
        if trace {
            defmt::trace!("CONTROL UPDATE TRACE - KF update: {} us, effort compute: {} us, control outputs: {} us, KF predict: {} us",
                kf_update_time.as_micros(),
                effort_time.as_micros(),
                control_outputs_time.as_micros(),
                kf_predict_time.as_micros(),
            );
        }
    }

    pub fn compute_effort_pose_control(&mut self, state_estimate: Vector6f, target_pose: Vector3f) {
        let global_accel_cmd = self.pid_controller.calculate(
            &target_pose, 
            &state_estimate.fixed_rows::<3>(0).into(),
            self.loop_period.as_micros() as f32 * 1e-6
        );
        let global_twist_cmd = state_estimate.fixed_rows::<3>(3) + self.body_accel_cmd * (self.loop_period.as_micros() as f32 * 1e-6);

        // // Calculate the optimal trajectory to the setpoint
        // let traj = ateam_controls::bangbang_trajectory::compute_optimal_bangbang_traj_3d_pose(
        //     state_estimate, target_pose,
        // );
        // // Calculate the acceleration needed to achieve the trajectory right now
        // let global_accel_cmd = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_accel_at_t(traj, 0.0);
        // // Calculate the twist that should be achieved at the next time step after applying this acceleration
        // let next_body_state = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_state_at_t(traj, state_estimate, 0.0, self.loop_period.as_micros() as f32 * 1e-6);
        // let global_twist_cmd = Vector3f::new(next_body_state[3], next_body_state[4], next_body_state[5]);

        // These torques are discretized by the loop rate, but in an ideal world, it would be a continuous command update to reach the next state wheel velocities as theta changes. However, the loop rate should be fast enough that the error due to a change in theta during each control period should be negligible, and the individual wheel velocities are achieved by the next control update
        self.body_twist_cmd = global_twist_cmd;
        self.body_accel_cmd = global_accel_cmd;
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * global_twist_cmd;
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * global_accel_cmd;
    }

    pub fn compute_effort_twist_control(&mut self, state_estimate: Vector6f, target_twist: Vector3f) {
        let init_twist: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let traj = ateam_controls::bangbang_trajectory::compute_optimal_bangbang_traj_3d_twist(
            init_twist, 
            target_twist
        );
        let next_state = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_state_at_t(traj, state_estimate, 0.0, self.loop_period.as_micros() as f32 * 1e-6);
        let global_twist_cmd: Vector3f = next_state.fixed_rows::<3>(3).into();
        let global_accel_cmd = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_accel_at_t(traj, 0.0);

        self.body_twist_cmd = global_twist_cmd;
        self.body_accel_cmd = global_accel_cmd;
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * global_twist_cmd;
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * global_accel_cmd;
    }

    pub fn compute_effort_accel_control(&mut self, state_estimate: Vector6f, target_accel: Vector3f) {
        let next_state = self.robot_model.a * state_estimate + self.robot_model.b * target_accel;
        let global_twist_cmd = next_state.fixed_rows::<3>(3).into();
        let global_accel_cmd = target_accel;

        self.body_twist_cmd = global_twist_cmd;
        self.body_accel_cmd = global_accel_cmd;
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * global_twist_cmd;
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * global_accel_cmd;
    }

    pub fn get_wheel_velocities(&self) -> Vector4f {
        self.wheel_vel_cmd
    }

    /// Get the 4 wheel torque commands in Nm
    pub fn get_wheel_torques(&self) -> Vector4f {
        self.wheel_torque_cmd
    }

    /// Get the 4 wheel currents in amps
    pub fn get_wheel_currents(&self) -> Vector4f {
        self.robot_model.torques_to_currents(self.wheel_torque_cmd)
    }

    pub fn get_control_debug_telem(&self) -> ExtendedTelemetry {
        self.debug_telemetry
    }
}

impl ParameterInterface for BodyController {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool {
        return self.has_name(param_cmd.parameter_name);
    }

    fn has_name(&self, param_name: ParameterName::Type) -> bool {
        return match param_name {
            ParameterName::KF_PROCESS_STD_POS_LINEAR => true,
            ParameterName::KF_PROCESS_STD_POS_ANGULAR => true,
            ParameterName::KF_PROCESS_STD_VEL_LINEAR => true,
            ParameterName::KF_PROCESS_STD_VEL_ANGULAR => true,
            ParameterName::KF_VISION_STD_LINEAR => true,
            ParameterName::KF_VISION_STD_ANGULAR => true,
            ParameterName::KF_ENCODER_STD_ANGULAR => true,
            ParameterName::KF_GYRO_STD_ANGULAR => true,
            ParameterName::KF_MAX_POS_LINEAR => true,
            ParameterName::KF_MAX_POS_ANGULAR => true,
            ParameterName::KF_MAX_VEL_LINEAR => true,
            ParameterName::KF_MAX_VEL_ANGULAR => true,
            ParameterName::PHYS_WHEEL_ANGLE_ALPHA => true,
            ParameterName::PHYS_WHEEL_ANGLE_BETA => true,
            ParameterName::PHYS_WHEEL_DISTANCE => true,
            ParameterName::PHYS_WHEEL_RADIUS => true,
            ParameterName::PHYS_BODY_MASS => true,
            ParameterName::PHYS_BODY_MOMENT_Z => true,
            ParameterName::PHYS_MOTOR_TORQUE_CONSTANT => true,
            ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => true,
            ParameterName::PIDII_X => true,
            ParameterName::PIDII_Y => true,
            ParameterName::PIDII_THETA => true,
            _ => false,
        };
    }

    fn apply_command(
        &mut self,
        param_cmd: &ParameterCommand,
    ) -> Result<ParameterCommand, ParameterCommand> {
        let mut reply_cmd = param_cmd.clone();

        // if we haven't been given an actionable command code, ignore the call
        if !(param_cmd.command_code == PCC_READ || param_cmd.command_code == PCC_WRITE) {
            defmt::warn!("asked to apply a command with out and actional command code");
            return Err(reply_cmd);
        }

        // if we've been asked to apply a command we don't have a key for it
        // error out
        if !self.has_name(param_cmd.parameter_name) {
            defmt::warn!(
                "asked to apply a command with a parameter name not managed by this module"
            );
            reply_cmd.command_code = PCC_NACK_INVALID_NAME;
            return Err(*param_cmd);
        }

        if param_cmd.command_code == PCC_READ {
            defmt::info!("Reading parameter {}", param_cmd.parameter_name);
            match param_cmd.parameter_name {
                ParameterName::KF_PROCESS_STD_POS_LINEAR
                | ParameterName::KF_PROCESS_STD_POS_ANGULAR
                | ParameterName::KF_PROCESS_STD_VEL_LINEAR
                | ParameterName::KF_PROCESS_STD_VEL_ANGULAR
                | ParameterName::KF_VISION_STD_LINEAR
                | ParameterName::KF_VISION_STD_ANGULAR
                | ParameterName::KF_ENCODER_STD_ANGULAR
                | ParameterName::KF_GYRO_STD_ANGULAR
                | ParameterName::KF_MAX_POS_LINEAR
                | ParameterName::KF_MAX_POS_ANGULAR
                | ParameterName::KF_MAX_VEL_LINEAR
                | ParameterName::KF_MAX_VEL_ANGULAR
                | ParameterName::PHYS_WHEEL_ANGLE_ALPHA
                | ParameterName::PHYS_WHEEL_ANGLE_BETA
                | ParameterName::PHYS_WHEEL_DISTANCE
                | ParameterName::PHYS_WHEEL_RADIUS
                | ParameterName::PHYS_BODY_MASS
                | ParameterName::PHYS_BODY_MOMENT_Z 
                | ParameterName::PHYS_MOTOR_TORQUE_CONSTANT
                | ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => {
                    reply_cmd.data_format = ParameterDataFormat::F32;
                    reply_cmd.data.f32_ = match param_cmd.parameter_name {
                        ParameterName::KF_PROCESS_STD_POS_LINEAR => self.robot_model.kf_params.process_noise_std_pos_linear,
                        ParameterName::KF_PROCESS_STD_POS_ANGULAR => self.robot_model.kf_params.process_noise_std_pos_angular,
                        ParameterName::KF_PROCESS_STD_VEL_LINEAR => self.robot_model.kf_params.process_noise_std_vel_linear,
                        ParameterName::KF_PROCESS_STD_VEL_ANGULAR => self.robot_model.kf_params.process_noise_std_vel_angular,
                        ParameterName::KF_VISION_STD_LINEAR => self.robot_model.kf_params.measurement_noise_std_vision_pos_linear,
                        ParameterName::KF_VISION_STD_ANGULAR => self.robot_model.kf_params.measurement_noise_std_vision_pos_angular,
                        ParameterName::KF_ENCODER_STD_ANGULAR => self.robot_model.kf_params.measurement_noise_std_encoder_vel_angular,
                        ParameterName::KF_GYRO_STD_ANGULAR => self.robot_model.kf_params.measurement_noise_std_gyro_vel_angular,
                        ParameterName::KF_MAX_POS_LINEAR => self.robot_model.kf_params.max_pos_linear,
                        ParameterName::KF_MAX_POS_ANGULAR => self.robot_model.kf_params.max_pos_angular,
                        ParameterName::KF_MAX_VEL_LINEAR => self.robot_model.kf_params.max_vel_linear,
                        ParameterName::KF_MAX_VEL_ANGULAR => self.robot_model.kf_params.max_vel_angular,
                        ParameterName::PHYS_WHEEL_ANGLE_ALPHA => self.robot_model.physical_params.alpha,
                        ParameterName::PHYS_WHEEL_ANGLE_BETA => self.robot_model.physical_params.beta,
                        ParameterName::PHYS_WHEEL_DISTANCE => self.robot_model.physical_params.l,
                        ParameterName::PHYS_WHEEL_RADIUS => self.robot_model.physical_params.r,
                        ParameterName::PHYS_BODY_MASS => self.robot_model.physical_params.mass,
                        ParameterName::PHYS_BODY_MOMENT_Z => self.robot_model.physical_params.iz,
                        ParameterName::PHYS_MOTOR_TORQUE_CONSTANT => self.robot_model.physical_params.motor_torque_constant,
                        ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => self.robot_model.physical_params.motor_efficiency_factor,
                        _ => unreachable!(),
                    };
                },
                ParameterName::PIDII_X
                | ParameterName::PIDII_Y
                | ParameterName::PIDII_THETA => {
                    reply_cmd.data_format = ParameterDataFormat::PID_LIMITED_INTEGRAL_F32;
                    let gain = self.pid_controller.get_gain();
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_X => 0,
                        ParameterName::PIDII_Y => 1,
                        ParameterName::PIDII_THETA => 2,
                        _ => unreachable!(),
                    };
                    reply_cmd.data.pidii_f32 = [gain[(row, 0)], gain[(row, 1)], gain[(row, 2)], gain[(row, 3)], gain[(row, 4)]];
                },
                _ => {
                    defmt::debug!("unimplemented key read in RobotController");
                    reply_cmd.command_code = PCC_NACK_INVALID_NAME;
                    return Err(reply_cmd);
                }
            };
            reply_cmd.command_code = PCC_ACK;
            return Ok(reply_cmd);
        } else if param_cmd.command_code == PCC_WRITE {
            defmt::info!("Writing parameter {}", param_cmd.parameter_name);
            match param_cmd.parameter_name {
                ParameterName::KF_PROCESS_STD_POS_LINEAR
                | ParameterName::KF_PROCESS_STD_POS_ANGULAR
                | ParameterName::KF_PROCESS_STD_VEL_LINEAR
                | ParameterName::KF_PROCESS_STD_VEL_ANGULAR
                | ParameterName::KF_VISION_STD_LINEAR
                | ParameterName::KF_VISION_STD_ANGULAR
                | ParameterName::KF_ENCODER_STD_ANGULAR
                | ParameterName::KF_GYRO_STD_ANGULAR
                | ParameterName::KF_MAX_POS_LINEAR
                | ParameterName::KF_MAX_POS_ANGULAR
                | ParameterName::KF_MAX_VEL_LINEAR
                | ParameterName::KF_MAX_VEL_ANGULAR => {
                    if param_cmd.data_format != ParameterDataFormat::F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_value = unsafe { param_cmd.data.f32_ };
                    let mut kf_params = self.robot_model.kf_params;
                    match param_cmd.parameter_name {
                        ParameterName::KF_PROCESS_STD_POS_LINEAR => kf_params.process_noise_std_pos_linear = write_value,
                        ParameterName::KF_PROCESS_STD_POS_ANGULAR => kf_params.process_noise_std_pos_angular = write_value,
                        ParameterName::KF_PROCESS_STD_VEL_LINEAR => kf_params.process_noise_std_vel_linear = write_value,
                        ParameterName::KF_PROCESS_STD_VEL_ANGULAR => kf_params.process_noise_std_vel_angular = write_value,
                        ParameterName::KF_VISION_STD_LINEAR => kf_params.measurement_noise_std_vision_pos_linear = write_value,
                        ParameterName::KF_VISION_STD_ANGULAR => kf_params.measurement_noise_std_vision_pos_angular = write_value,
                        ParameterName::KF_ENCODER_STD_ANGULAR => kf_params.measurement_noise_std_encoder_vel_angular = write_value,
                        ParameterName::KF_GYRO_STD_ANGULAR => kf_params.measurement_noise_std_gyro_vel_angular = write_value,
                        ParameterName::KF_MAX_POS_LINEAR => kf_params.max_pos_linear = write_value,
                        ParameterName::KF_MAX_POS_ANGULAR => kf_params.max_pos_angular = write_value,
                        ParameterName::KF_MAX_VEL_LINEAR => kf_params.max_vel_linear = write_value,
                        ParameterName::KF_MAX_VEL_ANGULAR => kf_params.max_vel_angular = write_value,
                        _ => unreachable!(),
                    }
                    self.robot_model.update_kf_params(kf_params);
                },
                ParameterName::PHYS_WHEEL_ANGLE_ALPHA
                | ParameterName::PHYS_WHEEL_ANGLE_BETA
                | ParameterName::PHYS_WHEEL_DISTANCE
                | ParameterName::PHYS_WHEEL_RADIUS
                | ParameterName::PHYS_BODY_MASS
                | ParameterName::PHYS_BODY_MOMENT_Z
                | ParameterName::PHYS_MOTOR_TORQUE_CONSTANT
                | ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => {
                    if param_cmd.data_format != ParameterDataFormat::F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_value = unsafe { param_cmd.data.f32_ };
                    let mut physical_params = self.robot_model.physical_params;
                    match param_cmd.parameter_name {
                        ParameterName::PHYS_WHEEL_ANGLE_ALPHA => physical_params.alpha = write_value,
                        ParameterName::PHYS_WHEEL_ANGLE_BETA => physical_params.beta = write_value,
                        ParameterName::PHYS_WHEEL_DISTANCE => physical_params.l = write_value,
                        ParameterName::PHYS_WHEEL_RADIUS => physical_params.r = write_value,
                        ParameterName::PHYS_BODY_MASS => physical_params.mass = write_value,
                        ParameterName::PHYS_BODY_MOMENT_Z => physical_params.iz = write_value,
                        ParameterName::PHYS_MOTOR_TORQUE_CONSTANT => physical_params.motor_torque_constant = write_value,
                        ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => physical_params.motor_efficiency_factor = write_value,
                        _ => unreachable!(),
                    }
                    self.robot_model.update_physical_params(physical_params);
                },
                ParameterName::PIDII_X
                | ParameterName::PIDII_Y
                | ParameterName::PIDII_THETA => {
                    if param_cmd.data_format != ParameterDataFormat::PID_LIMITED_INTEGRAL_F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_values = unsafe { param_cmd.data.pidii_f32 };
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_X => 0,
                        ParameterName::PIDII_Y => 1,
                        ParameterName::PIDII_THETA => 2,
                        _ => unreachable!(),
                    };
                    let mut gain = self.pid_controller.get_gain();
                    for col in 0..5 {
                        gain[(row, col)] = write_values[col];
                    }
                    self.pid_controller.set_gain(gain);
                    self.pid_controller.reset();
                    reply_cmd.data.pidii_f32 = write_values;
                },
                _ => {
                    defmt::debug!("unimplemented key write in RobotController");
                    reply_cmd.command_code = PCC_NACK_INVALID_NAME;
                    return Err(reply_cmd);
                }
            }
            reply_cmd.command_code = PCC_ACK;
        }

        return Ok(reply_cmd);
    }
}
