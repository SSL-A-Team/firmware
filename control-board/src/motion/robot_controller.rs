use crate::motion::constant_gain_kalman_filter::CgKalmanFilter;
use crate::motion::params::body_vel_filter_params::{
    CONTROL_INPUT, INIT_ESTIMATE_COV, KALMAN_GAIN, KF_NUM_CONTROL_INPUTS, KF_NUM_OBSERVATIONS,
    KF_NUM_STATES, OBSERVATION_MODEL, PROCESS_COV, STATE_TRANSITION,
};
use crate::motion::pid::PidController;
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{BodyControlMode, BodyControlTelemetry, ParameterCommandCode::*, ParameterDataFormat, ParameterName};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::{Vector1f, Vector3f, Vector4f, Vector5f, Vector6f, Vector8f};
use ateam_lib_stm32::drivers::imu;
use embassy_time::Instant;
use libm::{fabsf, sqrtf};
use nalgebra::{matrix, vector, Matrix1x5, Matrix3x5};

use ateam_common_packets::bindings::{ExtendedTelemetry, ParameterCommand};


pub struct BodyController {
    pub robot_model: RobotModel,
    pub pose_pid_controller: PidController<3>,
    /// [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_control_gain: Vector2f,
    /// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR] thresholds for when to recompute the trajectory
    pub traj_recompute_error: Vector4f,
    pub trajectory: Option<BangBangTraj3D>,
    pub trajectory_time: f32,
    pub trajectory_state: Vector6f,
    pub trajectory_params: TrajectoryParams,
    pub prev_body_cmd: Option<Vector3f>,
    pub body_twist_out: Vector3f,
    pub body_accel_out: Vector3f,
    pub body_accel_out_fric_comp: Vector3f,
    pub wheel_vel_out: Vector4f,
    pub wheel_torque_out: Vector4f,
    pub telemetry: BodyControlTelemetry,
    pub dt: f32,
}

impl<'a> BodyController {
    pub fn new(dt: f32) -> BodyController {
        let linear_pid_gains = Vector5f::new(115.0, 0.0, 2.75, 0.0, 0.0).transpose();
        let angular_pid_gains = Vector5f::new(250.0, 0.0, 10.0, 0.0, 0.0).transpose();
        BodyController {
            robot_model: RobotModel::new_from_default_params(dt)
                .expect("Failed to create RobotModel, check that default parameters are valid"),
            pose_pid_controller: PidController::<3>::from_gains_matrix(&Matrix3x5::zeros()),
            pose_control_gain: Vector2f::new(1.0, 1.0),
            traj_recompute_error: Vector4f::new(0.5, 1.0, 1.0, 2.0),
            trajectory: None,
            trajectory_state: Vector6f::zeros(),
            trajectory_params: TrajectoryParams::default(),
            trajectory_time: 0.0,
            prev_body_cmd: None,
            body_twist_out: Vector3f::default(),
            body_accel_out: Vector3f::default(),
            body_accel_out_fric_comp: Vector3f::default(),
            wheel_vel_out: Vector4f::default(),
            wheel_torque_out: Vector4f::default(),
            telemetry: Default::default(),
            dt,
        }
    }

    pub fn reset(&mut self) {
        self.robot_model.reset();
        self.pose_pid_controller.reset();
        self.trajectory = None;
        self.trajectory_state = Vector6f::default();
        self.trajectory_time = 0.0;
        self.prev_body_cmd = None;
        self.body_twist_out = Vector3f::default();
        self.body_accel_out = Vector3f::default();
        self.body_accel_out_fric_comp = Vector3f::default();
        self.wheel_vel_out = Vector4f::default();
        self.wheel_torque_out = Vector4f::default();
        self.telemetry = Default::default();
    }

    pub fn control_update(
        &mut self,
        body_cmd: Vector3f,
        body_control_mode: BodyControlMode::Type,
        vision_pose_meas: Vector3f,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
        imu_accel_x_meas: f32,
        imu_accel_y_meas: f32,
        trace: bool,
    ) {
        let mut start = Instant::now();

        let state_prediction = self.robot_model.get_state();

        self.state_update(
            vision_pose_meas,
            vision_update,
            wheel_vel_meas,
            imu_gyro_theta_meas,
        );

        let mut state_estimate = self.robot_model.get_state();

        // Deadzone the velocity estimate
        if fabsf(state_estimate[3]) < 0.05 {
            state_estimate[3] = 0.0;
        }
        if fabsf(state_estimate[4]) < 0.05 {
            state_estimate[4] = 0.0;
        }
        if fabsf(state_estimate[5]) < 0.05 {
            state_estimate[5] = 0.0;
        }

        if trace {
            defmt::trace!(
                "State Estimate: [{}, {}, {}, {}, {}, {}]",
                state_estimate.x,
                state_estimate.y,
                state_estimate.z,
                state_estimate.w,
                state_estimate.a,
                state_estimate.b,
            );
        }

        let kf_update_time = Instant::now() - start;
        start = Instant::now();

        if body_control_mode == BodyControlMode::BCM_GLOBAL_POSE {
            self.compute_effort_pose_control(state_estimate, body_cmd);
        } else if body_control_mode == BodyControlMode::BCM_GLOBAL_TWIST {
            self.compute_effort_twist_control(state_estimate, body_cmd);
        } else if body_control_mode == BodyControlMode::BCM_GLOBAL_ACCEL {
            self.compute_effort_accel_control(state_estimate, body_cmd);
        } else {
            self.body_twist_out = Vector3f::default();
            self.body_accel_out = Vector3f::default();
            self.body_accel_out_fric_comp = Vector3f::default();
            self.wheel_vel_out = Vector4f::default();
            self.wheel_torque_out = Vector4f::default();
        }

        let effort_time = Instant::now() - start;
        start = Instant::now();

        // defmt::trace!(
        //     "Wheel Velocity Cmds: {}, {}, {}, {}",
        //     self.wheel_vel_cmd.x,
        //     self.wheel_vel_cmd.y,
        //     self.wheel_vel_cmd.z,
        //     self.wheel_vel_cmd.w
        // );

        // Copy values to telemetry
        self.telemetry = BodyControlTelemetry {
            body_control_mode,
            _bitfield_align_1: Default::default(),
            _bitfield_1: BodyControlTelemetry::new_bitfield_1(
                vision_update as u8,
                Default::default(),
            ),
            _reserved2: Default::default(),
            imu_gyro: [0.0, 0.0, imu_gyro_theta_meas],
            imu_accel: [imu_accel_x_meas, imu_accel_y_meas, 0.0],
            vision_pose: vision_pose_meas.into(),
            body_cmd: body_cmd.into(),
            body_traj_pose: self.trajectory_state.fixed_rows::<3>(0).into(),
            body_traj_twist: self.trajectory_state.fixed_rows::<3>(3).into(),
            kf_body_pose_prediction: state_prediction.fixed_rows::<3>(0).into(),
            kf_body_twist_prediction: state_prediction.fixed_rows::<3>(3).into(),
            kf_body_pose_estimate: state_estimate.fixed_rows::<3>(0).into(),
            kf_body_twist_estimate: state_estimate.fixed_rows::<3>(3).into(),
            body_twist_u: self.body_twist_out.into(),
            body_accel_u: self.body_accel_out.into(),
            body_accel_u_fric_comp: self.body_accel_out_fric_comp.into(),
        };

        let control_outputs_time = Instant::now() - start;
        start = Instant::now();

        self.robot_model.kf_predict(self.body_accel_out);

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

    pub fn get_wheel_velocities(&self) -> Vector4f {
        self.wheel_vel_out
    }

    /// Get the 4 wheel torque commands in Nm
    pub fn get_wheel_torques(&self) -> Vector4f {
        self.wheel_torque_out
    }

    /// Get the 4 wheel currents in amps
    pub fn get_wheel_currents(&self) -> Vector4f {
        self.robot_model.torques_to_currents(self.wheel_torque_out)
    }

    pub fn get_control_debug_telem(&self) -> BodyControlTelemetry {
        self.telemetry
    }

    fn state_update(
        &mut self,
        vision_pose_meas: Vector3f,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
    ) {
        // Use EKF
        let measurement: Vector8f = vector![
            vision_pose_meas.x,
            vision_pose_meas.y,
            vision_pose_meas.z,
            wheel_vel_meas.x,
            wheel_vel_meas.y,
            wheel_vel_meas.z,
            wheel_vel_meas.w,
            imu_gyro_theta_meas,
        ];
        // TODO: consider what to do here for singular matrix cases - maybe skip the update and just use the prediction as the estimate for this cycle, or add some regularization to the covariance to prevent singularities?
        let res = self
            .robot_model
            .kf_update(measurement, !vision_update, false, false);
        if res.is_err() {
            defmt::error!(
                "Kalman filter update failed, skipping update: {}",
                res.err().unwrap() as i32
            );
        }
    }

    fn compute_effort_pose_control(&mut self, state_estimate: Vector6f, target_pose: Vector3f) {
        // Compute control output with control policy
        (self.body_twist_out, self.body_accel_out) =
            self.pose_bangbang_pid_control_policy(state_estimate, target_pose);
        // Compensate for modeled friction forces
        self.body_accel_out_fric_comp = self.body_accel_out - self.robot_model.i_inv * self.robot_model.compute_friction_force(self.body_twist_out);
        // Calculate wheel commands from body commands
        self.wheel_vel_out =
            self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_out;
        self.wheel_torque_out =
            self.robot_model.transform_accel2wheel(state_estimate.z) * self.body_accel_out_fric_comp;
    }

    fn compute_effort_twist_control(&mut self, state_estimate: Vector6f, target_twist: Vector3f) {
        // (self.body_twist_cmd, self.body_accel_cmd) = self.twist_bangbang_control(state_estimate, target_twist);
        (self.body_twist_cmd, self.body_accel_cmd) =
            self.twist_pid_control(state_estimate, target_twist);
        let compensated_accel = self.body_accel_cmd - self.robot_model.i_inv * self.robot_model.compute_friction_force(self.body_twist_cmd);
        self.wheel_vel_cmd =
            self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_cmd;
        self.wheel_torque_cmd =
            self.robot_model.transform_accel2wheel(state_estimate.z) * compensated_accel;
    }

    fn compute_effort_accel_control(&mut self, state_estimate: Vector6f, target_accel: Vector3f) {
        let next_state = self.robot_model.a * state_estimate + self.robot_model.b * target_accel;
        self.body_twist_cmd = next_state.fixed_rows::<3>(3).into();
        self.body_accel_cmd = target_accel;
        let compensated_accel = self.body_accel_cmd - self.robot_model.i_inv * self.robot_model.compute_friction_force(self.body_twist_cmd);
        self.wheel_vel_cmd =
            self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_cmd;
        self.wheel_torque_cmd =
            self.robot_model.transform_accel2wheel(state_estimate.z) * compensated_accel;
    }

    fn pose_bangbang_pid_control_policy(
        &mut self,
        state_estimate: Vector6f,
        target_pose: Vector3f,
    ) -> (Vector3f, Vector3f) {
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        // Compute trajectory to target pose if
        //   1) we don't have a trajectory yet
        //   2) the target pose has changed
        //   3) the current body configuration has strayed too far from the currently tracked trajectory
        if self.trajectory.is_none() || self.prev_body_cmd.is_none() || self.prev_body_cmd.unwrap() != target_pose ||
            (self.trajectory_state.x - pose_estimate.x).abs() > self.traj_recompute_error[(0, 0)] ||
            (self.trajectory_state.y - pose_estimate.y).abs() > self.traj_recompute_error[(0, 0)] ||
            (self.trajectory_state.z - pose_estimate.z).abs() > self.traj_recompute_error[(1, 0)] ||
            (self.trajectory_state[(3, 0)] - twist_estimate.x).abs() > self.traj_recompute_error[(2, 0)] ||
            (self.trajectory_state[(4, 0)] - twist_estimate.y).abs() > self.traj_recompute_error[(2, 0)] ||
            (self.trajectory_state[(5, 0)] - twist_estimate.z).abs() > self.traj_recompute_error[(3, 0)]
        {
            self.trajectory = Some(BangBangTraj3D::from_target_pose(
                state_estimate,
                target_pose,
                self.trajectory_params,
            ).expect("Failed to generate trajectory, check that trajectory params are valid"));
            self.trajectory_state = state_estimate;
            self.trajectory_time = 0.0;
        }

        // Compute feedforward as the tracked trajectory
        let ff = self.trajectory
            .as_ref()
            .expect("Trajectory should always be Some at this point since we set it if it was None above")
            .accel_at(self.trajectory_time)
            .expect("Trajectory should always have valid accel at current time");

        let target: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
        let fb = self.pose_pid_controller.calculate(
            &target,
            &pose_estimate,
            self.dt,
        );

        // Weighted sum of feedback and feedforward terms to calculate the accel command
        let global_accel_cmd = self.feedforward_gain * ff + self.feedback_gain * fb;

        let global_twist_cmd: Vector3f = (state_estimate.fixed_rows::<3>(3) + global_accel_cmd * self.dt).into();

        // Step trajectory forward
        let next_state = self.trajectory
            .expect("Trajectory should never be None at this point.")
            .state_at(self.trajectory_state, self.trajectory_time, self.trajectory_time + self.dt)
            .expect("Trajectory should always have valid state at current time + dt");

        self.trajectory_state = next_state;
        self.trajectory_time += self.dt;
        
        self.prev_body_cmd = Some(target_pose);

        (global_twist_cmd, global_accel_cmd)
    }

    fn twist_bangbang_control(
        &mut self,
        state_estimate: Vector6f,
        target_twist: Vector3f,
    ) -> (Vector3f, Vector3f) {
        let traj = BangBangTraj3D::from_target_twist(
            state_estimate.fixed_rows::<3>(3).into(),
            target_twist,
            self.trajectory_params,
        )
        .expect(
            "Failed to generate bang-bang trajectory, check that trajectory parameters are valid",
        );
        let next_state = traj
            .state_at(state_estimate, 0.0, self.dt)
            .expect("Bang-bang trajectory should always have a valid state at t=0.0 + dt");
        let global_twist_cmd: Vector3f = next_state.fixed_rows::<3>(3).into();
        let global_accel_cmd = traj
            .accel_at(0.0)
            .expect("Bang-bang trajectory should always have a valid accel at t=0.0");
        (global_twist_cmd, global_accel_cmd)
    }

    fn twist_pid_control(
        &mut self,
        state_estimate: Vector6f,
        target_twist: Vector3f,
    ) -> (Vector3f, Vector3f) {
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let body_twist_pid =
            self.twist_pid_controller
                .calculate(&target_twist, &twist_estimate, self.dt);
        let mut twist_cmd = target_twist + body_twist_pid;

        // Determine commanded body acceleration based on previous control output, and clamp and maintain the direction of acceleration.
        // NOTE: Using previous control output instead of estimate so that collision disturbances would not impact.
        let prev_twist_cmd = self.prev_body_cmd.unwrap_or(Vector3f::default());
        let mut accel_cmd = (twist_cmd - prev_twist_cmd) / self.dt;

        let max_accel_linear = self.trajectory_params.max_accel_linear;
        let max_accel_angular = self.trajectory_params.max_accel_angular;
        let max_vel_linear = self.trajectory_params.max_vel_linear;
        let max_vel_angular = self.trajectory_params.max_vel_angular;

        // Clamp acceleration: linear magnitude and angular independently
        let accel_linear_mag = sqrtf(accel_cmd.x * accel_cmd.x + accel_cmd.y * accel_cmd.y);
        if accel_linear_mag > max_accel_linear {
            let scale = max_accel_linear / accel_linear_mag;
            accel_cmd.x *= scale;
            accel_cmd.y *= scale;
        }
        accel_cmd.z = accel_cmd.z.clamp(-max_accel_angular, max_accel_angular);

        // Recompute twist from clamped acceleration
        twist_cmd = prev_twist_cmd + (accel_cmd * self.dt);

        // Clamp twist: linear magnitude and angular independently
        let twist_linear_mag = sqrtf(twist_cmd.x * twist_cmd.x + twist_cmd.y * twist_cmd.y);
        if twist_linear_mag > max_vel_linear {
            let scale = max_vel_linear / twist_linear_mag;
            twist_cmd.x *= scale;
            twist_cmd.y *= scale;
        }
        twist_cmd.z = twist_cmd.z.clamp(-max_vel_angular, max_vel_angular);

        // Recompute accel to stay consistent with the clamped twist
        accel_cmd = (twist_cmd - prev_twist_cmd) / self.dt;

        self.prev_body_cmd = Some(twist_cmd);

        (twist_cmd, accel_cmd)
    }
}

impl<'a> ParameterInterface for BodyController {
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
            ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_LINEAR => true,
            ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_ANGULAR => true,
            ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_LINEAR => true,
            ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_ANGULAR => true,
            ParameterName::PIDII_X => true,
            ParameterName::PIDII_Y => true,
            ParameterName::PIDII_THETA => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR => true,
            ParameterName::TRAJ_MAX_VEL_LINEAR => true,
            ParameterName::TRAJ_MAX_VEL_ANGULAR => true,
            ParameterName::TRAJ_MAX_ACCEL_LINEAR => true,
            ParameterName::TRAJ_MAX_ACCEL_ANGULAR => true,
            ParameterName::TRAJ_RECOMPUTE_ERROR => true,
            ParameterName::FEEDFORWARD_GAIN => true,
            ParameterName::FEEDBACK_GAIN => true,
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
                | ParameterName::KF_PROCESS_STD
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
                | ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR
                | ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_LINEAR
                | ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_ANGULAR
                | ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_LINEAR
                | ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_ANGULAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR
                | ParameterName::TRAJ_MAX_VEL_LINEAR
                | ParameterName::TRAJ_MAX_VEL_ANGULAR
                | ParameterName::TRAJ_MAX_ACCEL_LINEAR
                | ParameterName::TRAJ_MAX_ACCEL_ANGULAR
                | ParameterName::TRAJ_RECOMPUTE_ERROR => {
                | ParameterName::FEEDFORWARD_GAIN
                | ParameterName::FEEDBACK_GAIN => {
                    reply_cmd.data_format = ParameterDataFormat::F32;
                    reply_cmd.data.f32_ = match param_cmd.parameter_name {
                        ParameterName::KF_PROCESS_STD_POS_LINEAR => {
                            self.robot_model.kf_params.process_noise_std_pos_linear
                        }
                        ParameterName::KF_PROCESS_STD_POS_ANGULAR => {
                            self.robot_model.kf_params.process_noise_std_pos_angular
                        }
                        ParameterName::KF_PROCESS_STD_VEL_LINEAR => {
                            self.robot_model.kf_params.process_noise_std_vel_linear
                        }
                        ParameterName::KF_PROCESS_STD_VEL_ANGULAR => {
                            self.robot_model.kf_params.process_noise_std_vel_angular
                        }
                        ParameterName::KF_VISION_STD_LINEAR => {
                            self.robot_model
                                .kf_params
                                .measurement_noise_std_vision_pos_linear
                        }
                        ParameterName::KF_VISION_STD_ANGULAR => {
                            self.robot_model
                                .kf_params
                                .measurement_noise_std_vision_pos_angular
                        }
                        ParameterName::KF_ENCODER_STD_ANGULAR => {
                            self.robot_model
                                .kf_params
                                .measurement_noise_std_encoder_vel_angular
                        }
                        ParameterName::KF_GYRO_STD_ANGULAR => {
                            self.robot_model
                                .kf_params
                                .measurement_noise_std_gyro_vel_angular
                        }
                        ParameterName::KF_MAX_POS_LINEAR => {
                            self.robot_model.kf_params.max_pos_linear
                        }
                        ParameterName::KF_MAX_POS_ANGULAR => {
                            self.robot_model.kf_params.max_pos_angular
                        }
                        ParameterName::KF_MAX_VEL_LINEAR => {
                            self.robot_model.kf_params.max_vel_linear
                        }
                        ParameterName::KF_MAX_VEL_ANGULAR => {
                            self.robot_model.kf_params.max_vel_angular
                        }
                        ParameterName::PHYS_WHEEL_ANGLE_ALPHA => {
                            self.robot_model.physical_params.alpha
                        }
                        ParameterName::PHYS_WHEEL_ANGLE_BETA => {
                            self.robot_model.physical_params.beta
                        }
                        ParameterName::PHYS_WHEEL_DISTANCE => self.robot_model.physical_params.l,
                        ParameterName::PHYS_WHEEL_RADIUS => self.robot_model.physical_params.r,
                        ParameterName::PHYS_BODY_MASS => self.robot_model.physical_params.mass,
                        ParameterName::PHYS_BODY_MOMENT_Z => self.robot_model.physical_params.iz,
                        ParameterName::PHYS_MOTOR_TORQUE_CONSTANT => {
                            self.robot_model.physical_params.motor_torque_constant
                        }
                        ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => {
                            self.robot_model.physical_params.motor_efficiency_factor
                        }
                        ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_LINEAR => {
                            self.robot_model.physical_params.coulomb_friction_coefficient_linear
                        }
                        ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_ANGULAR => {
                            self.robot_model.physical_params.coulomb_friction_coefficient_angular
                        }
                        ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_LINEAR => {
                            self.robot_model.physical_params.viscous_friction_coefficient_linear
                        }
                        ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_ANGULAR => {
                            self.robot_model.physical_params.viscous_friction_coefficient_angular
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR => {
                            self.trajectory_params.allowable_error_pos_linear
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR => {
                            self.trajectory_params.allowable_error_pos_angular
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR => {
                            self.trajectory_params.allowable_error_vel_linear
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR => {
                            self.trajectory_params.allowable_error_vel_angular
                        }
                        ParameterName::TRAJ_MAX_VEL_LINEAR => self.trajectory_params.max_vel_linear,
                        ParameterName::TRAJ_MAX_VEL_ANGULAR => {
                            self.trajectory_params.max_vel_angular
                        }
                        ParameterName::TRAJ_MAX_ACCEL_LINEAR => {
                            self.trajectory_params.max_accel_linear
                        }
                        ParameterName::TRAJ_MAX_ACCEL_ANGULAR => {
                            self.trajectory_params.max_accel_angular
                        }
                        ParameterName::TRAJ_RECOMPUTE_ERROR => {
                            self.traj_recompute_error_vel_angular
                        }
                        ParameterName::FEEDFORWARD_GAIN => {
                            self.feedforward_gain
                        }
                        ParameterName::FEEDBACK_GAIN => {
                            self.feedback_gain
                        }
                        _ => unreachable!(),
                    };
                }
                ParameterName::PIDII_X | ParameterName::PIDII_Y | ParameterName::PIDII_THETA => {
                    reply_cmd.data_format = ParameterDataFormat::PID_LIMITED_INTEGRAL_F32;
                    let gain = self.pose_pid_controller.get_gain();
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_X => 0,
                        ParameterName::PIDII_Y => 1,
                        ParameterName::PIDII_THETA => 2,
                        _ => unreachable!(),
                    };
                    reply_cmd.data.pidii_f32 = [
                        gain[(row, 0)],
                        gain[(row, 1)],
                        gain[(row, 2)],
                        gain[(row, 3)],
                        gain[(row, 4)],
                    ];
                }
                ParameterName::PIDII_XD | ParameterName::PIDII_YD | ParameterName::PIDII_THETAD => {
                    reply_cmd.data_format = ParameterDataFormat::PID_LIMITED_INTEGRAL_F32;
                    let gain = self.twist_pid_controller.get_gain();
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_XD => 0,
                        ParameterName::PIDII_YD => 1,
                        ParameterName::PIDII_THETAD => 2,
                        _ => unreachable!(),
                    };
                    reply_cmd.data.pidii_f32 = [
                        gain[(row, 0)],
                        gain[(row, 1)],
                        gain[(row, 2)],
                        gain[(row, 3)],
                        gain[(row, 4)],
                    ];
                }
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
                        ParameterName::KF_PROCESS_STD_POS_LINEAR => {
                            kf_params.process_noise_std_pos_linear = write_value
                        }
                        ParameterName::KF_PROCESS_STD_POS_ANGULAR => {
                            kf_params.process_noise_std_pos_angular = write_value
                        }
                        ParameterName::KF_PROCESS_STD_VEL_LINEAR => {
                            kf_params.process_noise_std_vel_linear = write_value
                        }
                        ParameterName::KF_PROCESS_STD_VEL_ANGULAR => {
                            kf_params.process_noise_std_vel_angular = write_value
                        }
                        ParameterName::KF_VISION_STD_LINEAR => {
                            kf_params.measurement_noise_std_vision_pos_linear = write_value
                        }
                        ParameterName::KF_VISION_STD_ANGULAR => {
                            kf_params.measurement_noise_std_vision_pos_angular = write_value
                        }
                        ParameterName::KF_ENCODER_STD_ANGULAR => {
                            kf_params.measurement_noise_std_encoder_vel_angular = write_value
                        }
                        ParameterName::KF_GYRO_STD_ANGULAR => {
                            kf_params.measurement_noise_std_gyro_vel_angular = write_value
                        }
                        ParameterName::KF_MAX_POS_LINEAR => kf_params.max_pos_linear = write_value,
                        ParameterName::KF_MAX_POS_ANGULAR => {
                            kf_params.max_pos_angular = write_value
                        }
                        ParameterName::KF_MAX_VEL_LINEAR => kf_params.max_vel_linear = write_value,
                        ParameterName::KF_MAX_VEL_ANGULAR => {
                            kf_params.max_vel_angular = write_value
                        }
                        _ => unreachable!(),
                    }
                    self.robot_model.update_kf_params(kf_params);
                }
                ParameterName::PHYS_WHEEL_ANGLE_ALPHA
                | ParameterName::PHYS_WHEEL_ANGLE_BETA
                | ParameterName::PHYS_WHEEL_DISTANCE
                | ParameterName::PHYS_WHEEL_RADIUS
                | ParameterName::PHYS_BODY_MASS
                | ParameterName::PHYS_BODY_MOMENT_Z
                | ParameterName::PHYS_MOTOR_TORQUE_CONSTANT
                | ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR 
                | ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_LINEAR
                | ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_ANGULAR
                | ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_LINEAR
                | ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_ANGULAR => {
                    if param_cmd.data_format != ParameterDataFormat::F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_value = unsafe { param_cmd.data.f32_ };
                    let mut physical_params = self.robot_model.physical_params;
                    match param_cmd.parameter_name {
                        ParameterName::PHYS_WHEEL_ANGLE_ALPHA => {
                            physical_params.alpha = write_value
                        }
                        ParameterName::PHYS_WHEEL_ANGLE_BETA => physical_params.beta = write_value,
                        ParameterName::PHYS_WHEEL_DISTANCE => physical_params.l = write_value,
                        ParameterName::PHYS_WHEEL_RADIUS => physical_params.r = write_value,
                        ParameterName::PHYS_BODY_MASS => physical_params.mass = write_value,
                        ParameterName::PHYS_BODY_MOMENT_Z => physical_params.iz = write_value,
                        ParameterName::PHYS_MOTOR_TORQUE_CONSTANT => {
                            physical_params.motor_torque_constant = write_value
                        }
                        ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR => {
                            physical_params.motor_efficiency_factor = write_value
                        }
                        ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_LINEAR => {
                            physical_params.coulomb_friction_coefficient_linear = write_value
                        }
                        ParameterName::PHYS_COULOMB_FRICTION_COEFFICIENT_ANGULAR => {
                            physical_params.coulomb_friction_coefficient_angular = write_value
                        }
                        ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_LINEAR => {
                            physical_params.viscous_friction_coefficient_linear = write_value
                        }
                        ParameterName::PHYS_VISCOUS_FRICTION_COEFFICIENT_ANGULAR => {
                            physical_params.viscous_friction_coefficient_angular = write_value
                        }
                        _ => unreachable!(),
                    }
                    self.robot_model.update_physical_params(physical_params);
                }
                ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR
                | ParameterName::TRAJ_MAX_VEL_LINEAR
                | ParameterName::TRAJ_MAX_VEL_ANGULAR
                | ParameterName::TRAJ_MAX_ACCEL_LINEAR
                | ParameterName::TRAJ_MAX_ACCEL_ANGULAR => {
                    if param_cmd.data_format != ParameterDataFormat::F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_value = unsafe { param_cmd.data.f32_ };
                    match param_cmd.parameter_name {
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR => {
                            self.trajectory_params.allowable_error_pos_linear = write_value
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR => {
                            self.trajectory_params.allowable_error_pos_angular = write_value
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR => {
                            self.trajectory_params.allowable_error_vel_linear = write_value
                        }
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR => {
                            self.trajectory_params.allowable_error_vel_angular = write_value
                        }
                        ParameterName::TRAJ_MAX_VEL_LINEAR => {
                            self.trajectory_params.max_vel_linear = write_value
                        }
                        ParameterName::TRAJ_MAX_VEL_ANGULAR => {
                            self.trajectory_params.max_vel_angular = write_value
                        }
                        ParameterName::TRAJ_MAX_ACCEL_LINEAR => {
                            self.trajectory_params.max_accel_linear = write_value
                        }
                        ParameterName::TRAJ_MAX_ACCEL_ANGULAR => {
                            self.trajectory_params.max_accel_angular = write_value
                        }
                        _ => unreachable!(),
                    }
                }
                ParameterName::HYST_PID_ENTER_ERROR_POS_LINEAR
                | ParameterName::HYST_PID_ENTER_ERROR_POS_ANGULAR
                | ParameterName::HYST_PID_EXIT_ERROR_POS_LINEAR
                | ParameterName::HYST_PID_EXIT_ERROR_POS_ANGULAR
                | ParameterName::HYST_PID_ENTER_ERROR_VEL_LINEAR
                | ParameterName::HYST_PID_ENTER_ERROR_VEL_ANGULAR
                | ParameterName::HYST_PID_EXIT_ERROR_VEL_LINEAR
                | ParameterName::HYST_PID_EXIT_ERROR_VEL_ANGULAR => {
                    if param_cmd.data_format != ParameterDataFormat::F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_value = unsafe { param_cmd.data.f32_ };
                    match param_cmd.parameter_name {
                        ParameterName::HYST_PID_ENTER_ERROR_POS_LINEAR => {
                            self.pose_control_hysteresis.pid_enter_error_pos_linear = write_value
                        }
                        ParameterName::HYST_PID_ENTER_ERROR_POS_ANGULAR => {
                            self.pose_control_hysteresis.pid_enter_error_pos_angular = write_value
                        }
                        ParameterName::HYST_PID_EXIT_ERROR_POS_LINEAR => {
                            self.pose_control_hysteresis.pid_exit_error_pos_linear = write_value
                        }
                        ParameterName::HYST_PID_EXIT_ERROR_POS_ANGULAR => {
                            self.pose_control_hysteresis.pid_exit_error_pos_angular = write_value
                        }
                        ParameterName::HYST_PID_ENTER_ERROR_VEL_LINEAR => {
                            self.pose_control_hysteresis.pid_enter_error_vel_linear = write_value
                        }
                        ParameterName::HYST_PID_ENTER_ERROR_VEL_ANGULAR => {
                            self.pose_control_hysteresis.pid_enter_error_vel_angular = write_value
                        }
                        ParameterName::HYST_PID_EXIT_ERROR_VEL_LINEAR => {
                            self.pose_control_hysteresis.pid_exit_error_vel_linear = write_value
                        }
                        ParameterName::HYST_PID_EXIT_ERROR_VEL_ANGULAR => {
                            self.pose_control_hysteresis.pid_exit_error_vel_angular = write_value
                        }
                        _ => unreachable!(),
                    }
                }
                ParameterName::TRAJ_RECOMPUTE_ERROR_POS_LINEAR
                | ParameterName::TRAJ_RECOMPUTE_ERROR_VEL_LINEAR
                | ParameterName::TRAJ_RECOMPUTE_ERROR_POS_ANGULAR
                | ParameterName::TRAJ_RECOMPUTE_ERROR_VEL_ANGULAR
                | ParameterName::FEEDFORWARD_GAIN
                | ParameterName::FEEDBACK_GAIN => {
                    if param_cmd.data_format != ParameterDataFormat::F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_value = unsafe { param_cmd.data.f32_ };
                    match param_cmd.parameter_name {
                        ParameterName::TRAJ_RECOMPUTE_ERROR_POS_LINEAR => {
                            self.traj_recompute_error_pos_linear = write_value
                        }
                        ParameterName::TRAJ_RECOMPUTE_ERROR_VEL_LINEAR => {
                            self.traj_recompute_error_vel_linear = write_value
                        }
                        ParameterName::TRAJ_RECOMPUTE_ERROR_POS_ANGULAR => {
                            self.traj_recompute_error_pos_angular = write_value
                        }
                        ParameterName::TRAJ_RECOMPUTE_ERROR_VEL_ANGULAR => {
                            self.traj_recompute_error_vel_angular = write_value
                        }
                        ParameterName::FEEDFORWARD_GAIN => {
                            self.feedforward_gain = write_value
                        }
                        ParameterName::FEEDBACK_GAIN => {
                            self.feedback_gain = write_value
                        }
                        _ => unreachable!(),
                    }
                }
                ParameterName::PIDII_X | ParameterName::PIDII_Y | ParameterName::PIDII_THETA => {
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
                    let mut gain = self.pose_pid_controller.get_gain();
                    for col in 0..5 {
                        gain[(row, col)] = write_values[col];
                    }
                    self.pose_pid_controller.set_gain(gain);

                    let dimensional_controller = match param_cmd.parameter_name {
                        ParameterName::PIDII_X => &mut self.pose_pid_controller_x,
                        ParameterName::PIDII_Y => &mut self.pose_pid_controller_y,
                        ParameterName::PIDII_THETA => &mut self.pose_pid_controller_theta,
                        _ => unreachable!(),
                    };
                    let gain = matrix![write_values[0], write_values[1], write_values[2], write_values[3], write_values[4]];
                    dimensional_controller.set_gain(gain);

                    reply_cmd.data.pidii_f32 = write_values;
                }
                ParameterName::PIDII_XD | ParameterName::PIDII_YD | ParameterName::PIDII_THETAD => {
                    if param_cmd.data_format != ParameterDataFormat::PID_LIMITED_INTEGRAL_F32 {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                    let write_values = unsafe { param_cmd.data.pidii_f32 };
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_XD => 0,
                        ParameterName::PIDII_YD => 1,
                        ParameterName::PIDII_THETAD => 2,
                        _ => unreachable!(),
                    };
                    let mut gain = self.twist_pid_controller.get_gain();
                    for col in 0..5 {
                        gain[(row, col)] = write_values[col];
                    }
                    self.twist_pid_controller.set_gain(gain);
                    reply_cmd.data.pidii_f32 = write_values;
                }
                _ => {
                    defmt::debug!("unimplemented key write in RobotController");
                    reply_cmd.command_code = PCC_NACK_INVALID_NAME;
                    return Err(reply_cmd);
                }
            }
            reply_cmd.command_code = PCC_ACK;
            self.reset();
        }

        return Ok(reply_cmd);
    }
}

// /// Active control mode for pose control, used for bang-bang / PID hysteresis.
// #[derive(Clone, Copy, Debug, PartialEq)]
// pub enum PoseControlMode {
//     /// Bang-bang trajectory controller (used for large errors)
//     BangBang,
//     /// PID controller (used once error is small)
//     Pid,
// }

// /// Hysteresis thresholds for switching between bang-bang and PID control.
// #[derive(Clone, Copy, Debug)]
// pub struct PoseControlHysteresis {
//     /// Linear position error below which we switch from bang-bang to PID
//     pub pid_enter_error_pos_linear: f32,
//     /// Angular position error below which we switch from bang-bang to PID
//     pub pid_enter_error_pos_angular: f32,
//     /// Linear position error above which we switch from PID back to bang-bang
//     pub pid_exit_error_pos_linear: f32,
//     /// Angular position error above which we switch from PID back to bang-bang
//     pub pid_exit_error_pos_angular: f32,
//     /// Linear velocity error below which we switch from bang-bang to PID
//     pub pid_enter_error_vel_linear: f32,
//     /// Angular velocity error below which we switch from bang-bang to PID
//     pub pid_enter_error_vel_angular: f32,
//     /// Linear velocity error above which we switch from PID back to bang-bang
//     pub pid_exit_error_vel_linear: f32,
//     /// Angular velocity error above which we switch from PID back to bang-bang
//     pub pid_exit_error_vel_angular: f32,
// }

// impl Default for PoseControlHysteresis {
//     fn default() -> Self {
//         PoseControlHysteresis {
//             pid_enter_error_pos_linear: 0.05, // meters
//             pid_enter_error_pos_angular: 0.2, // radians
//             pid_exit_error_pos_linear: 0.1,   // meters (larger than enter for hysteresis)
//             pid_exit_error_pos_angular: 0.4,  // radians (larger than enter for hysteresis)
//             pid_enter_error_vel_linear: 0.2,  // m/s
//             pid_enter_error_vel_angular: 0.5, // rad/s
//             pid_exit_error_vel_linear: 0.4,   // m/s (larger than enter for hysteresis)
//             pid_exit_error_vel_angular: 0.8,  // rad/s (larger than enter for hysteresis)
//         }
//     }
// }

// fn pose_pid_control(
//     &mut self,
//     state_estimate: Vector6f,
//     target_pose: Vector3f,
// ) -> (Vector3f, Vector3f) {
//     let global_accel_cmd = self.pose_pid_controller.calculate(
//         &target_pose,
//         &state_estimate.fixed_rows::<3>(0).into(),
//         self.dt,
//     );
//     let global_twist_cmd = state_estimate.fixed_rows::<3>(3) + self.body_accel_out * self.dt;
//     (global_twist_cmd, global_accel_cmd)
// }

// fn pose_bangbang_control(
//     &mut self,
//     state_estimate: Vector6f,
//     target_pose: Vector3f,
// ) -> (Vector3f, Vector3f) {
//     // Calculate the optimal trajectory to the setpoint
//     let traj = BangBangTraj3D::from_target_pose(
//         state_estimate,
//         target_pose,
//         self.trajectory_params,
//     )
//     .expect(
//         "Failed to generate bang-bang trajectory, check that trajectory parameters are valid",
//     );
//     // Calculate the acceleration needed to achieve the trajectory right now
//     let global_accel_cmd = traj
//         .accel_at(0.0)
//         .expect("Bang-bang trajectory should always have a valid accel at t=0.0");
//     // Calculate the twist that should be achieved at the next time step after applying this acceleration
//     let next_body_state = traj
//         .state_at(state_estimate, 0.0, self.dt)
//         .expect("Bang-bang trajectory should always have a valid state at t=0.0 + dt");
//     let global_twist_cmd =
//         Vector3f::new(next_body_state[3], next_body_state[4], next_body_state[5]);
//     (global_twist_cmd, global_accel_cmd)
// }

// fn pose_dualmode_control(
//     &mut self,
//     state_estimate: Vector6f,
//     target_pose: Vector3f,
// ) -> (Vector3f, Vector3f) {
//     let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
//     let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

//     let pose_error: Vector3f = target_pose - pose_estimate;
//     let linear_error = sqrtf(pose_error.x * pose_error.x + pose_error.y * pose_error.y);
//     let angular_error = pose_error.z.abs();
//     // Target pose implies zero velocity, so velocity error is the current velocity magnitude
//     let linear_vel_error =
//         sqrtf(twist_estimate.x * twist_estimate.x + twist_estimate.y * twist_estimate.y);
//     let angular_vel_error = twist_estimate.z.abs();

//     let hyst = &self.pose_control_hysteresis;

//     // Dual-mode switching with hysteresis:
//     //  - Switch to PID when both position and velocity errors drop below the enter thresholds
//     //  - Switch back to bang-bang when any position or velocity error rises above the exit thresholds
//     match self.pose_control_mode_x {
//         PoseControlMode::BangBang => {
//             if linear_error < hyst.pid_enter_error_pos_linear
//                 && angular_error < hyst.pid_enter_error_pos_angular
//                 && linear_vel_error < hyst.pid_enter_error_vel_linear
//                 && angular_vel_error < hyst.pid_enter_error_vel_angular
//             {
//                 self.pose_control_mode_x = PoseControlMode::Pid;
//                 self.pose_control_mode_y = PoseControlMode::Pid;
//                 self.pose_control_mode_theta = PoseControlMode::Pid;
//                 self.pose_pid_controller.reset();
//             }
//         }
//         PoseControlMode::Pid => {
//             if linear_error > hyst.pid_exit_error_pos_linear
//                 || angular_error > hyst.pid_exit_error_pos_angular
//                 || linear_vel_error > hyst.pid_exit_error_vel_linear
//                 || angular_vel_error > hyst.pid_exit_error_vel_angular
//             {
//                 self.pose_control_mode_x = PoseControlMode::BangBang;
//                 self.pose_control_mode_y = PoseControlMode::BangBang;
//                 self.pose_control_mode_theta = PoseControlMode::BangBang;
//             }
//         }
//     }

//     match self.pose_control_mode_x {
//         PoseControlMode::BangBang => self.pose_bangbang_control(state_estimate, target_pose),
//         PoseControlMode::Pid => self.pose_pid_control(state_estimate, target_pose),
//     }
// }

// fn pose_dimensional_dualmode_control(
//     &mut self,
//     state_estimate: Vector6f,
//     target_pose: Vector3f,
// ) -> (Vector3f, Vector3f) {
//     let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
//     let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

//     // Compute trajectory to target pose if
//     //   1) we don't have a trajectory yet
//     //   2) the target pose has changed
//     //   3) the current body configuration has strayed too far from the currently tracked trajectory
//     if self.trajectory.is_none() || self.prev_body_cmd.is_none() || self.prev_body_cmd.unwrap() != target_pose ||
//         (self.trajectory_state.x - pose_estimate.x).abs() > self.traj_recompute_error_pos_linear ||
//         (self.trajectory_state.y - pose_estimate.y).abs() > self.traj_recompute_error_pos_linear ||
//         (self.trajectory_state.z - pose_estimate.z).abs() > self.traj_recompute_error_pos_angular ||
//         (self.trajectory_state[(3, 0)] - twist_estimate.x).abs() > self.traj_recompute_error_vel_linear ||
//         (self.trajectory_state[(4, 0)] - twist_estimate.y).abs() > self.traj_recompute_error_vel_linear ||
//         (self.trajectory_state[(5, 0)] - twist_estimate.z).abs() > self.traj_recompute_error_vel_angular
//     {
//         self.trajectory = Some(BangBangTraj3D::from_target_pose(
//             state_estimate,
//             target_pose,
//             self.trajectory_params,
//         ).expect("Failed to generate trajectory, check that trajectory params are valid"));
//         self.trajectory_state = state_estimate;
//         self.trajectory_time = 0.0;
//     }

//     // For each dimension, determine whether to use bang-bang or PID control
//     match self.pose_control_mode_x {
//         PoseControlMode::BangBang => {
//             if (pose_estimate.x - target_pose.x).abs() < self.pose_control_hysteresis.pid_enter_error_pos_linear &&
//                 twist_estimate.x.abs() < self.pose_control_hysteresis.pid_enter_error_vel_linear
//             {
//                 self.pose_control_mode_x = PoseControlMode::Pid;
//                 self.pose_pid_controller_x.reset();
//             }
//         }
//         PoseControlMode::Pid => {
//             if (pose_estimate.x - target_pose.x).abs() > self.pose_control_hysteresis.pid_exit_error_pos_linear ||
//                 twist_estimate.x.abs() > self.pose_control_hysteresis.pid_exit_error_vel_linear
//             {
//                 self.pose_control_mode_x = PoseControlMode::BangBang;
//             }
//         }
//     }
//     match self.pose_control_mode_y {
//         PoseControlMode::BangBang => {
//             if (pose_estimate.y - target_pose.y).abs() < self.pose_control_hysteresis.pid_enter_error_pos_linear &&
//                 twist_estimate.y.abs() < self.pose_control_hysteresis.pid_enter_error_vel_linear
//             {
//                 self.pose_control_mode_y = PoseControlMode::Pid;
//                 self.pose_pid_controller_y.reset();
//             }
//         }
//         PoseControlMode::Pid => {
//             if (pose_estimate.y - target_pose.y).abs() > self.pose_control_hysteresis.pid_exit_error_pos_linear ||
//                 twist_estimate.y.abs() > self.pose_control_hysteresis.pid_exit_error_vel_linear
//             {
//                 self.pose_control_mode_y = PoseControlMode::BangBang;
//             }
//         }
//     }
//     match self.pose_control_mode_theta {
//         PoseControlMode::BangBang => {
//             if (pose_estimate.z - target_pose.z).abs() < self.pose_control_hysteresis.pid_enter_error_pos_angular &&
//                 twist_estimate.z.abs() < self.pose_control_hysteresis.pid_enter_error_vel_angular
//             {
//                 self.pose_control_mode_theta = PoseControlMode::Pid;
//                 self.pose_pid_controller_theta.reset();
//             }
//         }
//         PoseControlMode::Pid => {
//             if (pose_estimate.z - target_pose.z).abs() > self.pose_control_hysteresis.pid_exit_error_pos_angular ||
//                 twist_estimate.z.abs() > self.pose_control_hysteresis.pid_exit_error_vel_angular
//             {
//                 self.pose_control_mode_theta = PoseControlMode::BangBang;
//             }
//         }
//     }

//     let mut global_accel_cmd = Vector3f::default();

//     // For each dimension, compute control output based on the active control mode
//     match self.pose_control_mode_x {
//         PoseControlMode::BangBang => {
//             global_accel_cmd.x = self.trajectory
//                 .as_ref()
//                 .expect("Trajectory should always be Some at this point since we set it if it was None above")
//                 .accel_at(self.trajectory_time)
//                 .expect("Trajectory should always have valid accel at current time")
//                 .x;
//         },
//         PoseControlMode::Pid => {
//             // let target_x: Vector1f = target_pose.fixed_rows::<1>(0).into();
//             let target_x: Vector1f = self.trajectory_state.fixed_rows::<1>(0).into();
//             let current_x: Vector1f = pose_estimate.fixed_rows::<1>(0).into();
//             global_accel_cmd.x = self.pose_pid_controller_x.calculate(
//                 &target_x, 
//                 &current_x,
//                 self.dt
//             )[(0, 0)];
//         },
//     }
//     match self.pose_control_mode_y {
//         PoseControlMode::BangBang => {
//             global_accel_cmd.y = self.trajectory
//                 .as_ref()
//                 .expect("Trajectory should always be Some at this point since we set it if it was None above")
//                 .accel_at(self.trajectory_time)
//                 .expect("Trajectory should always have valid accel at current time")
//                 .y;
//         },
//         PoseControlMode::Pid => {
//             // let target_y: Vector1f = target_pose.fixed_rows::<1>(1).into();
//             let target_y: Vector1f = self.trajectory_state.fixed_rows::<1>(1).into();
//             let current_y: Vector1f = pose_estimate.fixed_rows::<1>(1).into();
//             global_accel_cmd.y = self.pose_pid_controller_y.calculate(
//                 &target_y, 
//                 &current_y,
//                 self.dt
//             )[(0, 0)];
//         },
//     }
//     match self.pose_control_mode_theta {
//         PoseControlMode::BangBang => {
//             global_accel_cmd.z = self.trajectory
//                 .as_ref()
//                 .expect("Trajectory should always be Some at this point since we set it if it was None above")
//                 .accel_at(self.trajectory_time)
//                 .expect("Trajectory should always have valid accel at current time")
//                 .z;
//         },
//         PoseControlMode::Pid => {
//             // let target_theta: Vector1f = target_pose.fixed_rows::<1>(2).into();
//             let target_theta: Vector1f = self.trajectory_state.fixed_rows::<1>(2).into();
//             let current_theta: Vector1f = pose_estimate.fixed_rows::<1>(2).into();
//             global_accel_cmd.z = self.pose_pid_controller_theta.calculate(
//                 &target_theta, 
//                 &current_theta,
//                 self.dt
//             )[(0, 0)];
//         },
//     }

//     let global_twist_cmd = state_estimate.fixed_rows::<3>(3) + global_accel_cmd * self.dt;

//     // Step trajectory forward
//     self.trajectory_state = self.trajectory
//         .as_ref()
//         .expect("Trajectory should always be Some at this point since we set it if it was None above")
//         .state_at(self.trajectory_state, self.trajectory_time, self.trajectory_time + self.dt)
//         .expect("Trajectory should always have valid state at current time + dt");
//     self.trajectory_time += self.dt;
    
//     self.prev_body_cmd = Some(target_pose);

//     (global_twist_cmd, global_accel_cmd)
// }