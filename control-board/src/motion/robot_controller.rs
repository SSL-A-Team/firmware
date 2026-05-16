use crate::motion::pid::PidController;
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{BasicControl, BodyControlExtendedTelemetry, BodyControlTelemetry, ExtendedGlobalAccelerationTelemetry, ExtendedGlobalPositionTelemetry, ExtendedGlobalVelocityTelemetry, ExtendedLocalAccelerationTelemetry, ExtendedLocalVelocityTelemetry, ParameterCommandCode::*, ParameterDataFormat, ParameterName};
use ateam_common_packets::radio::{SkillCommand, SkillExtendedTelemetry};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::{z_rotation_mat, ControlsError, Vector2f, Vector3f, Vector4f, Vector5f, Vector6f, Vector8f};
use embassy_time::Instant;
use libm::{fabsf, remainderf};
use core::f32::consts::PI;
use nalgebra::{vector, Matrix3x5};

use ateam_common_packets::bindings::ParameterCommand;


/// Time (seconds) without a vision update after which vision is considered "inactive".
const VISION_ACTIVE_TIMEOUT_S: f32 = 0.2;

pub struct BodyController {
    pub robot_model: RobotModel,
    pub pose_pid_controller: PidController<3>,
    pub twist_pid_controller: PidController<3>,
    /// [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_control_gain: Vector2f,
    /// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR] thresholds for when to recompute the trajectory
    pub traj_recompute_error: Vector4f,
    pub trajectory: Option<BangBangTraj3D>,
    pub trajectory_time: f32,
    pub trajectory_state: Vector6f,
    pub prev_body_cmd: Option<Vector3f>,
    pub body_twist_out: Vector3f,
    pub body_accel_out: Vector3f,
    pub body_accel_out_fric_comp: Vector3f,
    pub wheel_vel_out: Vector4f,
    pub wheel_torque_out: Vector4f,
    pub telemetry: BodyControlTelemetry,
    pub debug_telemetry: BodyControlExtendedTelemetry,
    pub dt: f32,
    pub first_vision_received: bool,
    pub time_since_vision_update_s: f32,
}

impl BodyController {
    pub fn new(dt: f32) -> BodyController {

        let linear_pose_pid_gains = Vector5f::new(125.0, 0.5, 10.0, -1.0, 1.0).transpose();
        let angular_pose_pid_gains = Vector5f::new(125.0, 0.5, 15.0, -1.0, 1.0).transpose();
        let pose_pid_gains = Matrix3x5::from_rows(&[linear_pose_pid_gains, linear_pose_pid_gains, angular_pose_pid_gains]);

        let linear_twist_pid_gains = Vector5f::new(100.0, 0.0, 0.0, 0.0, 0.0).transpose();
        let angular_twist_pid_gains = Vector5f::new(20.0, 0.0, 0.0, 0.0, 0.0).transpose();
        let twist_pid_gains = Matrix3x5::from_rows(&[linear_twist_pid_gains, linear_twist_pid_gains, angular_twist_pid_gains]);

        BodyController {
            robot_model: RobotModel::new_from_default_params(dt)
                .expect("Failed to create RobotModel, check that default parameters are valid"),
            pose_pid_controller: PidController::<3>::from_gains_matrix(&pose_pid_gains),
            twist_pid_controller: PidController::<3>::from_gains_matrix(&twist_pid_gains),
            pose_control_gain: Vector2f::new(1.0, 1.0),
            traj_recompute_error: Vector4f::new(0.5, 1.0, 1.0, 2.0),
            trajectory: None,
            trajectory_state: Vector6f::zeros(),
            trajectory_time: 0.0,
            prev_body_cmd: None,
            body_twist_out: Vector3f::default(),
            body_accel_out: Vector3f::default(),
            body_accel_out_fric_comp: Vector3f::default(),
            wheel_vel_out: Vector4f::default(),
            wheel_torque_out: Vector4f::default(),
            telemetry: Default::default(),
            debug_telemetry: Default::default(),
            dt,
            first_vision_received: false,
            time_since_vision_update_s: VISION_ACTIVE_TIMEOUT_S + 1.0,  // Set to higher than timeout
        }
    }

    pub fn reset(&mut self) {
        self.robot_model.reset();
        self.pose_pid_controller.reset();
        self.twist_pid_controller.reset();
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
        self.debug_telemetry = Default::default();
        self.first_vision_received = false;
        self.time_since_vision_update_s = VISION_ACTIVE_TIMEOUT_S + 1.0;  // Set to higher than timeout
    }

    pub fn vision_active(&self) -> bool {
        self.first_vision_received && self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S
    }

    pub fn control_update(
        &mut self,
        last_command: BasicControl,
        vision_pose_meas: Vector3f,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
        imu_accel_x_meas: f32,
        imu_accel_y_meas: f32,
        trace: bool,
    ) -> Result<(), ControlsError> {
        let t_start = Instant::now();

        // Working in global frame, unless variable is specified as local

        if vision_update {
            if !self.first_vision_received {
                self.robot_model.kf_set_pose(vision_pose_meas);
                self.first_vision_received = true;
            }
            self.time_since_vision_update_s = 0.0;
        } else {
            if self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S {
                self.time_since_vision_update_s += self.dt;  // Only add when less than timeout
            }
        }

        let state_prediction = self.robot_model.get_state();

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

        self.robot_model
            .kf_update(measurement, !vision_update, false, false)?;
        let mut state_estimate = self.robot_model.get_state();

        // Deadzone the velocity estimate
        if fabsf(state_estimate[3]) < 0.1 {
            state_estimate[3] = 0.0;
        }
        if fabsf(state_estimate[4]) < 0.1 {
            state_estimate[4] = 0.0;
        }
        if fabsf(state_estimate[5]) < 0.2 {
            state_estimate[5] = 0.0;
        }

        let t_after_kf_update = Instant::now();

        let body_twist_out;
        let body_accel_out;
        let skill_telem;
        match last_command.get_skill_command() {
            SkillCommand::GlobalPosition(gpos_cmd) => {
                let traj_params = TrajectoryParams {
                    max_vel_linear: gpos_cmd.max_linear_vel,
                    max_vel_angular: gpos_cmd.max_angular_vel,
                    max_accel_linear: gpos_cmd.max_linear_acc,
                    max_accel_angular: gpos_cmd.max_angular_acc,
                };
                (body_twist_out, body_accel_out) = self.global_pose_bangbang_pid_control_policy(state_estimate, gpos_cmd.as_vec3f(), traj_params)?;
                skill_telem = SkillExtendedTelemetry::GlobalPosition(ExtendedGlobalPositionTelemetry { cmd_echo: gpos_cmd });
            }
            SkillCommand::GlobalVelocity(gvel_cmd) => {
                let traj_params = TrajectoryParams {
                    max_vel_linear: 0.0,
                    max_vel_angular: 0.0,
                    max_accel_linear: gvel_cmd.max_linear_acc,
                    max_accel_angular: gvel_cmd.max_angular_acc,
                };
                (body_twist_out, body_accel_out) = self.global_twist_control_policy(state_estimate, gvel_cmd.as_vec3f(), traj_params)?;
                skill_telem = SkillExtendedTelemetry::GlobalVelocity(ExtendedGlobalVelocityTelemetry { cmd_echo: gvel_cmd });
            }
            SkillCommand::LocalVelocity(lvel_cmd) => {
                let traj_params = TrajectoryParams {
                    max_vel_linear: 0.0,
                    max_vel_angular: 0.0,
                    max_accel_linear: lvel_cmd.max_linear_acc,
                    max_accel_angular: lvel_cmd.max_angular_acc,
                };
                (body_twist_out, body_accel_out) = self.local_twist_control_policy(state_estimate, lvel_cmd.as_vec3f(), traj_params)?;
                skill_telem = SkillExtendedTelemetry::LocalVelocity(ExtendedLocalVelocityTelemetry { cmd_echo: lvel_cmd });
            }
            SkillCommand::GlobalAcceleration(gacc_cmd) => {
                (body_twist_out, body_accel_out) = self.global_accel_control_policy(state_estimate, gacc_cmd.as_vec3f());
                skill_telem = SkillExtendedTelemetry::GlobalAcceleration(ExtendedGlobalAccelerationTelemetry { cmd_echo: gacc_cmd });
            }
            SkillCommand::LocalAcceleration(lacc_cmd) => {
                (body_twist_out, body_accel_out) = self.local_accel_control_policy(state_estimate, lacc_cmd.as_vec3f());
                skill_telem = SkillExtendedTelemetry::LocalAcceleration(ExtendedLocalAccelerationTelemetry { cmd_echo: lacc_cmd });
            }
            _ => {
                (body_twist_out, body_accel_out) = (Vector3f::zeros(), Vector3f::zeros());
                skill_telem = SkillExtendedTelemetry::Off;
            }
        };

        self.body_twist_out = body_twist_out;
        self.body_accel_out = body_accel_out;

        // Compensate for modeled friction forces

        // // Using the target twist will activate coulomb friction compensation even when the robot is stationary, which helps to overcome static friction. Using the estimated twist would cause the controller to not apply any torque when the robot is stationary due to zero estimated velocity, which would make it unable to overcome static friction to start moving. However, it's causing small oscillations at the setpoint.
        // self.body_accel_out_fric_comp = self.body_accel_out - self.robot_model.i_inv * self.robot_model.compute_friction_force(self.body_twist_out);

        // Using the deadzoned estimated twist will prevent oscillations at the setpoint, but will fail to use coulomb friction compensation to overcome static friction when at rest.
        self.body_accel_out_fric_comp = self.body_accel_out - self.robot_model.i_inv * self.robot_model.compute_friction_force(state_estimate.fixed_rows::<3>(3).into());

        // Calculate wheel commands from body commands
        self.wheel_vel_out =
            self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_out;
        self.wheel_torque_out =
            self.robot_model.transform_accel2wheel(state_estimate.z) * self.body_accel_out_fric_comp;

        let t_after_effort = Instant::now();

        // Copy values to debug telemetry
        self.debug_telemetry = BodyControlExtendedTelemetry {
            _bitfield_align_1: Default::default(),
            _bitfield_1: BodyControlExtendedTelemetry::new_bitfield_1(
                vision_update as u8,
                Default::default(),
            ),
            _reserved2: Default::default(),
            imu_gyro: [0.0, 0.0, imu_gyro_theta_meas],
            imu_accel: [imu_accel_x_meas, imu_accel_y_meas, 0.0],
            vision_pose: vision_pose_meas.into(),
            body_traj_pos: self.trajectory_state.fixed_rows::<3>(0).into(),
            body_traj_vel: self.trajectory_state.fixed_rows::<3>(3).into(),
            kf_body_pos_prediction: state_prediction.fixed_rows::<3>(0).into(),
            kf_body_vel_prediction: state_prediction.fixed_rows::<3>(3).into(),
            kf_body_pos_estimate: state_estimate.fixed_rows::<3>(0).into(),
            kf_body_vel_estimate: state_estimate.fixed_rows::<3>(3).into(),
            body_vel_u: self.body_twist_out.into(),
            body_accel_u: self.body_accel_out.into(),
            body_accel_u_fric_comp: self.body_accel_out_fric_comp.into(),
            ..self.debug_telemetry
        };
        self.debug_telemetry.set_skill_telemetry(skill_telem);

        let t_after_telem = Instant::now();

        self.robot_model.kf_predict(self.body_accel_out);

        let t_after_kf_predict = Instant::now();
        if trace {
            defmt::trace!("CONTROL UPDATE TRACE - KF update: {} us, effort compute: {} us, control outputs: {} us, KF predict: {} us",
                (t_after_kf_update - t_start).as_micros(),
                (t_after_effort - t_after_kf_update).as_micros(),
                (t_after_telem - t_after_effort).as_micros(),
                (t_after_kf_predict - t_after_telem).as_micros(),
            );
        }

        Ok(())
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

    pub fn get_control_telem(&self) -> BodyControlTelemetry {
        self.telemetry
    }

    pub fn get_control_debug_telem(&self) -> BodyControlExtendedTelemetry {
        self.debug_telemetry
    }

    fn global_accel_control_policy(&mut self, state_estimate: Vector6f, target_accel: Vector3f) -> (Vector3f, Vector3f) {
        let next_state = self.robot_model.a * state_estimate + self.robot_model.b * target_accel;
        let twist_out = next_state.fixed_rows::<3>(3).into();
        let accel_out = target_accel;
        (twist_out, accel_out)
    }

    fn local_accel_control_policy(&mut self, state_estimate: Vector6f, local_target_accel: Vector3f) -> (Vector3f, Vector3f) {
        let target_accel = z_rotation_mat(state_estimate.z) * local_target_accel;
        self.global_accel_control_policy(state_estimate, target_accel)
    }

    fn local_twist_control_policy(&mut self, state_estimate: Vector6f, local_target_twist: Vector3f, traj_params: TrajectoryParams) -> Result<(Vector3f, Vector3f), ControlsError> {
        let target_twist = z_rotation_mat(state_estimate.z) * local_target_twist;
        self.global_twist_control_policy(state_estimate, target_twist, traj_params)
    }

    fn global_twist_control_policy(&mut self, state_estimate: Vector6f, target_twist: Vector3f, traj_params: TrajectoryParams) -> Result<(Vector3f, Vector3f), ControlsError> {
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        // Compute trajectory to target twist if
        //   1) we don't have a trajectory yet
        //   2) the target twist has changed
        //   3) the current twist has strayed too far from the currently tracked trajectory
        if self.trajectory.is_none() || self.prev_body_cmd.is_none() || self.prev_body_cmd.unwrap() != target_twist ||
            (self.trajectory_state[(3, 0)] - twist_estimate.x).abs() > self.traj_recompute_error[(2, 0)] ||
            (self.trajectory_state[(4, 0)] - twist_estimate.y).abs() > self.traj_recompute_error[(2, 0)] ||
            (self.trajectory_state[(5, 0)] - twist_estimate.z).abs() > self.traj_recompute_error[(3, 0)]
        {
            self.trajectory = Some(BangBangTraj3D::from_target_twist(
                twist_estimate,
                target_twist,
                traj_params,
            )?);
            self.trajectory_state = state_estimate;
            self.trajectory_time = 0.0;
        }

        // Feedforward: acceleration from the tracked trajectory
        let ff = self.trajectory
            .as_ref()
            .expect("Trajectory should always be Some at this point since we set it if it was None above")
            .accel_at(self.trajectory_time)
            .expect("Trajectory should always have valid accel at current time");

        // Feedback: PID on twist error between tracked trajectory twist and estimated twist
        let target_traj_twist: Vector3f = self.trajectory_state.fixed_rows::<3>(3).into();
        let fb = self.twist_pid_controller.calculate(
            &target_traj_twist,
            &twist_estimate,
            self.dt,
        );

        // Weighted sum of feedforward and feedback
        // TODO: turn into params
        let twist_control_gain_ff = 1.0;
        let twist_control_gain_fb = 1.0;
        let accel_out = twist_control_gain_ff * ff + twist_control_gain_fb * fb;

        let twist_out: Vector3f = (state_estimate.fixed_rows::<3>(3) + accel_out * self.dt).into();

        // Step trajectory forward
        let next_state = self.trajectory
            .expect("Trajectory should never be None at this point.")
            .state_at(self.trajectory_state, self.trajectory_time, self.trajectory_time + self.dt)
            .expect("Trajectory should always have valid state at current time + dt");

        self.trajectory_state = next_state;
        self.trajectory_time += self.dt;

        self.prev_body_cmd = Some(target_twist);

        Ok((twist_out, accel_out))
    }

    fn global_pose_bangbang_pid_control_policy(
        &mut self,
        state_estimate: Vector6f,
        target_pose: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        // Vision required for pose control
        if !self.vision_active() {
            self.trajectory = None;
            self.trajectory_state = Vector6f::zeros();
            self.trajectory_time = 0.0;
            self.prev_body_cmd = None;
            return Ok((Vector3f::zeros(), Vector3f::zeros()));
        }

        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        // Compute trajectory to target pose if
        //   1) we don't have a trajectory yet
        //   2) the target pose has changed
        //   3) the current body configuration has strayed too far from the currently tracked trajectory
        if self.trajectory.is_none() || self.prev_body_cmd.is_none() || self.prev_body_cmd.unwrap() != target_pose ||
            (self.trajectory_state.x - pose_estimate.x).abs() > self.traj_recompute_error[(0, 0)] ||
            (self.trajectory_state.y - pose_estimate.y).abs() > self.traj_recompute_error[(0, 0)] ||
            remainderf(self.trajectory_state.z - pose_estimate.z, 2.0 * PI).abs() > self.traj_recompute_error[(1, 0)] ||
            (self.trajectory_state[(3, 0)] - twist_estimate.x).abs() > self.traj_recompute_error[(2, 0)] ||
            (self.trajectory_state[(4, 0)] - twist_estimate.y).abs() > self.traj_recompute_error[(2, 0)] ||
            (self.trajectory_state[(5, 0)] - twist_estimate.z).abs() > self.traj_recompute_error[(3, 0)]
        {
            self.trajectory = Some(BangBangTraj3D::from_target_pose(
                state_estimate,
                target_pose,
                traj_params,
            )?);
            self.trajectory_state = state_estimate;
            self.trajectory_time = 0.0;
        }

        // Compute feedforward as the tracked trajectory
        let ff = self.trajectory
            .as_ref()
            .expect("Trajectory should always be Some at this point since we set it if it was None above")
            .accel_at(self.trajectory_time)
            .expect("Trajectory should always have valid accel at current time");

        let mut target: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
        // Wrap target theta so that (target.z - pose_estimate.z) lies in [-π, π],
        target.z = pose_estimate.z + remainderf(target.z - pose_estimate.z, 2.0 * PI);
        let target_twist: Vector3f = self.trajectory_state.fixed_rows::<3>(3).into();
        let twist_error = target_twist - twist_estimate;
        let fb = self.pose_pid_controller.calculate_with_derivative(
            &target,
            &pose_estimate,
            &twist_error,
            self.dt,
        );

        // Weighted sum of feedback and feedforward terms to calculate the accel command
        let accel_out = self.pose_control_gain[0] * ff + self.pose_control_gain[1] * fb;

        let twist_out: Vector3f = (state_estimate.fixed_rows::<3>(3) + accel_out * self.dt).into();

        // Step trajectory forward
        let next_state = self.trajectory
            .expect("Trajectory should never be None at this point.")
            .state_at(self.trajectory_state, self.trajectory_time, self.trajectory_time + self.dt)
            .expect("Trajectory should always have valid state at current time + dt");

        self.trajectory_state = next_state;
        self.trajectory_time += self.dt;
        
        self.prev_body_cmd = Some(target_pose);

        Ok((twist_out, accel_out))
    }

    /// Returns the expected data format for a parameter name handled by BodyController,
    /// or None if the name is not recognized.
    fn expected_format(name: ParameterName::Type) -> Option<ParameterDataFormat::Type> {
        match name {
            ParameterName::KF_PROCESS_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MEASUREMENT_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MAX_STATE => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_WHEEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_INERTIA => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_MOTOR_MODEL => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_FRICTION_MODEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_CONTROL_GAIN => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRAJ_RECOMPUTE_ERROR => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_PIDII_LINEAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_ANGULAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::TWIST_FB_PIDII_LINEAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::TWIST_FB_PIDII_ANGULAR => Some(ParameterDataFormat::VEC5_F32),
            _ => None,
        }
    }

    fn read_param(&self, name: ParameterName::Type, reply: &mut ParameterCommand) {
        let kf = &self.robot_model.kf_params;
        let phys = &self.robot_model.physical_params;
        match name {
            ParameterName::KF_PROCESS_STD => {
                reply.data.vec4_f32 = [
                    kf.process_noise_std_pos_linear,
                    kf.process_noise_std_pos_angular,
                    kf.process_noise_std_vel_linear,
                    kf.process_noise_std_vel_angular,
                ];
            }
            ParameterName::KF_MEASUREMENT_STD => {
                reply.data.vec4_f32 = [
                    kf.measurement_noise_std_vision_pos_linear,
                    kf.measurement_noise_std_vision_pos_angular,
                    kf.measurement_noise_std_encoder_vel_angular,
                    kf.measurement_noise_std_gyro_vel_angular,
                ];
            }
            ParameterName::KF_MAX_STATE => {
                reply.data.vec4_f32 = [
                    kf.max_pos_linear,
                    kf.max_pos_angular,
                    kf.max_vel_linear,
                    kf.max_vel_angular,
                ];
            }
            ParameterName::PHYS_WHEEL => {
                reply.data.vec4_f32 = [phys.alpha, phys.beta, phys.l, phys.r];
            }
            ParameterName::PHYS_INERTIA => {
                reply.data.vec2_f32 = [phys.mass, phys.iz];
            }
            ParameterName::PHYS_MOTOR_MODEL => {
                reply.data.vec2_f32 = [phys.motor_torque_constant, phys.motor_efficiency_factor];
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                reply.data.vec4_f32 = [
                    phys.coulomb_friction_coefficient_linear,
                    phys.coulomb_friction_coefficient_angular,
                    phys.viscous_friction_coefficient_linear,
                    phys.viscous_friction_coefficient_angular,
                ];
            }
            ParameterName::POSE_CONTROL_GAIN => {
                reply.data.vec2_f32 = [self.pose_control_gain[0], self.pose_control_gain[1]];
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                reply.data.vec4_f32 = [
                    self.traj_recompute_error[0],
                    self.traj_recompute_error[1],
                    self.traj_recompute_error[2],
                    self.traj_recompute_error[3],
                ];
            }
            ParameterName::POSE_FB_PIDII_LINEAR
            | ParameterName::POSE_FB_PIDII_ANGULAR => {
                let gain = self.pose_pid_controller.get_gain();
                let row = match name {
                    ParameterName::POSE_FB_PIDII_LINEAR => 0,
                    _ => 2,
                };
                reply.data.vec5_f32 = [
                    gain[(row, 0)],
                    gain[(row, 1)],
                    gain[(row, 2)],
                    gain[(row, 3)],
                    gain[(row, 4)],
                ];
            }
            ParameterName::TWIST_FB_PIDII_LINEAR
            | ParameterName::TWIST_FB_PIDII_ANGULAR => {
                let gain = self.twist_pid_controller.get_gain();
                let row = match name {
                    ParameterName::TWIST_FB_PIDII_LINEAR => 0,
                    _ => 2,
                };
                reply.data.vec5_f32 = [
                    gain[(row, 0)],
                    gain[(row, 1)],
                    gain[(row, 2)],
                    gain[(row, 3)],
                    gain[(row, 4)],
                ];
            }
            _ => unreachable!(),
        }
    }

    fn write_param(&mut self, cmd: &ParameterCommand) {
        match cmd.parameter_name {
            ParameterName::KF_PROCESS_STD => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut kf = self.robot_model.kf_params;
                kf.process_noise_std_pos_linear = v[0];
                kf.process_noise_std_pos_angular = v[1];
                kf.process_noise_std_vel_linear = v[2];
                kf.process_noise_std_vel_angular = v[3];
                self.robot_model.update_kf_params(kf);
            }
            ParameterName::KF_MEASUREMENT_STD => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut kf = self.robot_model.kf_params;
                kf.measurement_noise_std_vision_pos_linear = v[0];
                kf.measurement_noise_std_vision_pos_angular = v[1];
                kf.measurement_noise_std_encoder_vel_angular = v[2];
                kf.measurement_noise_std_gyro_vel_angular = v[3];
                self.robot_model.update_kf_params(kf);
            }
            ParameterName::KF_MAX_STATE => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut kf = self.robot_model.kf_params;
                kf.max_pos_linear = v[0];
                kf.max_pos_angular = v[1];
                kf.max_vel_linear = v[2];
                kf.max_vel_angular = v[3];
                self.robot_model.update_kf_params(kf);
            }
            ParameterName::PHYS_WHEEL => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut p = self.robot_model.physical_params;
                p.alpha = v[0];
                p.beta = v[1];
                p.l = v[2];
                p.r = v[3];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::PHYS_INERTIA => {
                let v = unsafe { cmd.data.vec2_f32 };
                let mut p = self.robot_model.physical_params;
                p.mass = v[0];
                p.iz = v[1];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::PHYS_MOTOR_MODEL => {
                let v = unsafe { cmd.data.vec2_f32 };
                let mut p = self.robot_model.physical_params;
                p.motor_torque_constant = v[0];
                p.motor_efficiency_factor = v[1];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut p = self.robot_model.physical_params;
                p.coulomb_friction_coefficient_linear = v[0];
                p.coulomb_friction_coefficient_angular = v[1];
                p.viscous_friction_coefficient_linear = v[2];
                p.viscous_friction_coefficient_angular = v[3];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::POSE_CONTROL_GAIN => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.pose_control_gain = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.traj_recompute_error = Vector4f::new(v[0], v[1], v[2], v[3]);
            }
            ParameterName::POSE_FB_PIDII_LINEAR
            | ParameterName::POSE_FB_PIDII_ANGULAR => {
                let v = unsafe { cmd.data.vec5_f32 };
                let mut gain = self.pose_pid_controller.get_gain();
                if cmd.parameter_name == ParameterName::POSE_FB_PIDII_LINEAR {
                    for col in 0..5 {
                        gain[(0, col)] = v[col];
                        gain[(1, col)] = v[col];
                    }
                } else {
                    for col in 0..5 {
                        gain[(2, col)] = v[col];
                    }
                }
                self.pose_pid_controller.set_gain(gain);
            }
            ParameterName::TWIST_FB_PIDII_LINEAR
            | ParameterName::TWIST_FB_PIDII_ANGULAR => {
                let v = unsafe { cmd.data.vec5_f32 };
                let mut gain = self.twist_pid_controller.get_gain();
                if cmd.parameter_name == ParameterName::TWIST_FB_PIDII_LINEAR {
                    for col in 0..5 {
                        gain[(0, col)] = v[col];
                        gain[(1, col)] = v[col];
                    }
                } else {
                    for col in 0..5 {
                        gain[(2, col)] = v[col];
                    }
                }
                self.twist_pid_controller.set_gain(gain);
            }
            _ => unreachable!(),
        };
        self.reset();
    }
}

impl ParameterInterface for BodyController {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool {
        self.has_name(param_cmd.parameter_name)
    }

    fn has_name(&self, param_name: ParameterName::Type) -> bool {
        Self::expected_format(param_name).is_some()
    }

    fn apply_command(
        &mut self,
        param_cmd: &ParameterCommand,
    ) -> Result<ParameterCommand, ParameterCommand> {
        let mut reply = *param_cmd;

        if param_cmd.command_code != PCC_READ && param_cmd.command_code != PCC_WRITE {
            defmt::warn!("asked to apply a command without an actionable command code");
            return Err(reply);
        }

        let fmt = match Self::expected_format(param_cmd.parameter_name) {
            Some(f) => f,
            None => {
                defmt::warn!("unexpected parameter name {}, cannot apply command", param_cmd.parameter_name);
                reply.command_code = PCC_NACK_INVALID_NAME;
                return Err(reply);
            }
        };

        if param_cmd.command_code == PCC_READ {
            defmt::info!("Reading parameter {}", param_cmd.parameter_name);
            reply.data_format = fmt;
            self.read_param(param_cmd.parameter_name, &mut reply);
        } else {
            defmt::info!("Writing parameter {}", param_cmd.parameter_name);
            if param_cmd.data_format != fmt {
                reply.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                return Err(reply);
            }
            self.write_param(param_cmd);
        }

        reply.command_code = PCC_ACK;
        Ok(reply)
    }
}
