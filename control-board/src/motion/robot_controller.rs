use crate::motion::pid::PidController;
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{BodyControlMode, BodyControlTelemetry, ParameterCommandCode::*, ParameterDataFormat, ParameterName};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::{Vector2f, Vector3f, Vector4f, Vector5f, Vector6f, Vector8f, z_rotation_mat};
use embassy_time::Instant;
use libm::{fabsf, remainderf, sqrtf};
use core::f32::consts::PI;
use nalgebra::{vector, Matrix3x5};

use ateam_common_packets::bindings::ParameterCommand;


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

impl BodyController {
    pub fn new(dt: f32) -> BodyController {

        let linear_pose_pid_gains = Vector5f::new(115.0, 0.0, 2.75, 0.0, 0.0).transpose();
        let angular_pose_pid_gains = Vector5f::new(250.0, 0.0, 10.0, 0.0, 0.0).transpose();
        let pose_pid_gains = Matrix3x5::from_rows(&[linear_pose_pid_gains, linear_pose_pid_gains, angular_pose_pid_gains]);

        let linear_twist_pid_gains = Vector5f::new(0.0, 0.0, 0.0, 0.0, 0.0).transpose();
        let angular_twist_pid_gains = Vector5f::new(0.0, 0.0, 0.0, 0.0, 0.0).transpose();
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

        // Working in global frame, unless variable is specified as local

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

        (self.body_twist_out, self.body_accel_out) = match body_control_mode {
            BodyControlMode::BCM_GLOBAL_POSE => {
                self.global_pose_bangbang_pid_control_policy(state_estimate, body_cmd)
            }
            BodyControlMode::BCM_GLOBAL_TWIST => {
                todo!()
            }
            BodyControlMode::BCM_LOCAL_TWIST => {
                self.local_twist_control_policy(state_estimate, body_cmd)
            }
            BodyControlMode::BCM_GLOBAL_ACCEL => {
                self.global_accel_control_policy(state_estimate, body_cmd)
            }
            BodyControlMode::BCM_LOCAL_ACCEL => {
                self.local_accel_control_policy(state_estimate, body_cmd)
            }
            _ => {
                if body_control_mode != BodyControlMode::BCM_OFF {
                    defmt::error!("Received command with unrecognized control mode: {}", body_control_mode as i32);
                }
                (Vector3f::zeros(), Vector3f::zeros())
            }
        };

        // Compensate for modeled friction forces
        self.body_accel_out_fric_comp = self.body_accel_out - self.robot_model.i_inv * self.robot_model.compute_friction_force(self.body_twist_out);
        // Calculate wheel commands from body commands
        self.wheel_vel_out =
            self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_out;
        self.wheel_torque_out =
            self.robot_model.transform_accel2wheel(state_estimate.z) * self.body_accel_out_fric_comp;

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

    fn local_twist_control_policy(&mut self, state_estimate: Vector6f, local_target_twist: Vector3f) -> (Vector3f, Vector3f) {
        let target_twist = z_rotation_mat(state_estimate.z) * local_target_twist;
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let body_twist_pid =
            self.twist_pid_controller
                .calculate(&target_twist, &twist_estimate, self.dt);
        let mut twist_out = target_twist + body_twist_pid;

        // Determine commanded body acceleration based on previous control output, and clamp and maintain the direction of acceleration.
        // NOTE: Using previous control output instead of estimate so that collision disturbances would not impact.
        let prev_twist_out = self.prev_body_cmd.unwrap_or(Vector3f::default());
        let mut accel_out = (twist_out - prev_twist_out) / self.dt;

        let max_accel_linear = self.trajectory_params.max_accel_linear;
        let max_accel_angular = self.trajectory_params.max_accel_angular;
        let max_vel_linear = self.trajectory_params.max_vel_linear;
        let max_vel_angular = self.trajectory_params.max_vel_angular;

        // Clamp acceleration: linear magnitude and angular independently
        let accel_linear_mag = sqrtf(accel_out.x * accel_out.x + accel_out.y * accel_out.y);
        if accel_linear_mag > max_accel_linear {
            let scale = max_accel_linear / accel_linear_mag;
            accel_out.x *= scale;
            accel_out.y *= scale;
        }
        accel_out.z = accel_out.z.clamp(-max_accel_angular, max_accel_angular);

        // Recompute twist from clamped acceleration
        twist_out = prev_twist_out + (accel_out * self.dt);

        // Clamp twist: linear magnitude and angular independently
        let twist_linear_mag = sqrtf(twist_out.x * twist_out.x + twist_out.y * twist_out.y);
        if twist_linear_mag > max_vel_linear {
            let scale = max_vel_linear / twist_linear_mag;
            twist_out.x *= scale;
            twist_out.y *= scale;
        }
        twist_out.z = twist_out.z.clamp(-max_vel_angular, max_vel_angular);

        // Recompute accel to stay consistent with the clamped twist
        accel_out = (twist_out - prev_twist_out) / self.dt;

        self.prev_body_cmd = Some(twist_out);

        (twist_out, accel_out)
    }

    fn global_twist_control_policy(&mut self, state_estimate: Vector6f, target_accel: Vector3f) -> (Vector3f, Vector3f, f32) {
        todo!()
    }

    fn global_pose_bangbang_pid_control_policy(
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
            remainderf(self.trajectory_state.z - pose_estimate.z, 2.0 * PI).abs() > self.traj_recompute_error[(1, 0)] ||
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

        (twist_out, accel_out)
    }

    // fn twist_bangbang_control(
    //     &mut self,
    //     state_estimate: Vector6f,
    //     target_twist: Vector3f,
    // ) -> (Vector3f, Vector3f) {
    //     let traj = BangBangTraj3D::from_target_twist(
    //         state_estimate.fixed_rows::<3>(3).into(),
    //         target_twist,
    //         self.trajectory_params,
    //     )
    //     .expect(
    //         "Failed to generate bang-bang trajectory, check that trajectory parameters are valid",
    //     );
    //     let next_state = traj
    //         .state_at(state_estimate, 0.0, self.dt)
    //         .expect("Bang-bang trajectory should always have a valid state at t=0.0 + dt");
    //     let global_twist_out: Vector3f = next_state.fixed_rows::<3>(3).into();
    //     let global_accel_out = traj
    //         .accel_at(0.0)
    //         .expect("Bang-bang trajectory should always have a valid accel at t=0.0");
    //     (global_twist_out, global_accel_out)
    // }

//     fn twist_pid_control(
//         &mut self,
//         state_estimate: Vector6f,
//         target_twist: Vector3f,
//     ) -> (Vector3f, Vector3f) {
//         let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();
//         let body_twist_pid =
//             self.twist_pid_controller
//                 .calculate(&target_twist, &twist_estimate, self.dt);
//         let mut twist_out = target_twist + body_twist_pid;

//         // Determine commanded body acceleration based on previous control output, and clamp and maintain the direction of acceleration.
//         // NOTE: Using previous control output instead of estimate so that collision disturbances would not impact.
//         let prev_twist_out = self.prev_body_cmd.unwrap_or(Vector3f::default());
//         let mut accel_out = (twist_out - prev_twist_out) / self.dt;

//         let max_accel_linear = self.trajectory_params.max_accel_linear;
//         let max_accel_angular = self.trajectory_params.max_accel_angular;
//         let max_vel_linear = self.trajectory_params.max_vel_linear;
//         let max_vel_angular = self.trajectory_params.max_vel_angular;

//         // Clamp acceleration: linear magnitude and angular independently
//         let accel_linear_mag = sqrtf(accel_out.x * accel_out.x + accel_out.y * accel_out.y);
//         if accel_linear_mag > max_accel_linear {
//             let scale = max_accel_linear / accel_linear_mag;
//             accel_out.x *= scale;
//             accel_out.y *= scale;
//         }
//         accel_out.z = accel_out.z.clamp(-max_accel_angular, max_accel_angular);

//         // Recompute twist from clamped acceleration
//         twist_out = prev_twist_out + (accel_out * self.dt);

//         // Clamp twist: linear magnitude and angular independently
//         let twist_linear_mag = sqrtf(twist_out.x * twist_out.x + twist_out.y * twist_out.y);
//         if twist_linear_mag > max_vel_linear {
//             let scale = max_vel_linear / twist_linear_mag;
//             twist_out.x *= scale;
//             twist_out.y *= scale;
//         }
//         twist_out.z = twist_out.z.clamp(-max_vel_angular, max_vel_angular);

//         // Recompute accel to stay consistent with the clamped twist
//         accel_out = (twist_out - prev_twist_out) / self.dt;

//         self.prev_body_cmd = Some(twist_out);

//         (twist_out, accel_out)
//     }

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
            ParameterName::TRAJ_MAX => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_PIDII_X => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_Y => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_THETA => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::TWIST_FB_PIDII_X => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::TWIST_FB_PIDII_Y => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::TWIST_FB_PIDII_THETA => Some(ParameterDataFormat::VEC5_F32),
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
            ParameterName::TRAJ_MAX => {
                reply.data.vec4_f32 = [
                    self.trajectory_params.max_vel_linear,
                    self.trajectory_params.max_vel_angular,
                    self.trajectory_params.max_accel_linear,
                    self.trajectory_params.max_accel_angular,
                ];
            }
            ParameterName::POSE_FB_PIDII_X
            | ParameterName::POSE_FB_PIDII_Y
            | ParameterName::POSE_FB_PIDII_THETA => {
                let gain = self.pose_pid_controller.get_gain();
                let row = match name {
                    ParameterName::POSE_FB_PIDII_X => 0,
                    ParameterName::POSE_FB_PIDII_Y => 1,
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
            ParameterName::TWIST_FB_PIDII_X
            | ParameterName::TWIST_FB_PIDII_Y
            | ParameterName::TWIST_FB_PIDII_THETA => {
                let gain = self.twist_pid_controller.get_gain();
                let row = match name {
                    ParameterName::TWIST_FB_PIDII_X => 0,
                    ParameterName::TWIST_FB_PIDII_Y => 1,
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
            ParameterName::TRAJ_MAX => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.trajectory_params.max_vel_linear = v[0];
                self.trajectory_params.max_vel_angular = v[1];
                self.trajectory_params.max_accel_linear = v[2];
                self.trajectory_params.max_accel_angular = v[3];
            }
            ParameterName::POSE_FB_PIDII_X
            | ParameterName::POSE_FB_PIDII_Y
            | ParameterName::POSE_FB_PIDII_THETA => {
                let v = unsafe { cmd.data.vec5_f32 };
                let row = match cmd.parameter_name {
                    ParameterName::POSE_FB_PIDII_X => 0,
                    ParameterName::POSE_FB_PIDII_Y => 1,
                    _ => 2,
                };
                let mut gain = self.pose_pid_controller.get_gain();
                for col in 0..5 {
                    gain[(row, col)] = v[col];
                }
                self.pose_pid_controller.set_gain(gain);
            }
            ParameterName::TWIST_FB_PIDII_X
            | ParameterName::TWIST_FB_PIDII_Y
            | ParameterName::TWIST_FB_PIDII_THETA => {
                let v = unsafe { cmd.data.vec5_f32 };
                let row = match cmd.parameter_name {
                    ParameterName::TWIST_FB_PIDII_X => 0,
                    ParameterName::TWIST_FB_PIDII_Y => 1,
                    _ => 2,
                };
                let mut gain = self.twist_pid_controller.get_gain();
                for col in 0..5 {
                    gain[(row, col)] = v[col];
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
//     let global_accel_out = self.pose_pid_controller.calculate(
//         &target_pose,
//         &state_estimate.fixed_rows::<3>(0).into(),
//         self.dt,
//     );
//     let global_twist_out = state_estimate.fixed_rows::<3>(3) + self.body_accel_out * self.dt;
//     (global_twist_out, global_accel_out)
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
//     let global_accel_out = traj
//         .accel_at(0.0)
//         .expect("Bang-bang trajectory should always have a valid accel at t=0.0");
//     // Calculate the twist that should be achieved at the next time step after applying this acceleration
//     let next_body_state = traj
//         .state_at(state_estimate, 0.0, self.dt)
//         .expect("Bang-bang trajectory should always have a valid state at t=0.0 + dt");
//     let global_twist_out =
//         Vector3f::new(next_body_state[3], next_body_state[4], next_body_state[5]);
//     (global_twist_out, global_accel_out)
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

//     let mut global_accel_out = Vector3f::default();

//     // For each dimension, compute control output based on the active control mode
//     match self.pose_control_mode_x {
//         PoseControlMode::BangBang => {
//             global_accel_out.x = self.trajectory
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
//             global_accel_out.x = self.pose_pid_controller_x.calculate(
//                 &target_x, 
//                 &current_x,
//                 self.dt
//             )[(0, 0)];
//         },
//     }
//     match self.pose_control_mode_y {
//         PoseControlMode::BangBang => {
//             global_accel_out.y = self.trajectory
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
//             global_accel_out.y = self.pose_pid_controller_y.calculate(
//                 &target_y, 
//                 &current_y,
//                 self.dt
//             )[(0, 0)];
//         },
//     }
//     match self.pose_control_mode_theta {
//         PoseControlMode::BangBang => {
//             global_accel_out.z = self.trajectory
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
//             global_accel_out.z = self.pose_pid_controller_theta.calculate(
//                 &target_theta, 
//                 &current_theta,
//                 self.dt
//             )[(0, 0)];
//         },
//     }

//     let global_twist_out = state_estimate.fixed_rows::<3>(3) + global_accel_out * self.dt;

//     // Step trajectory forward
//     self.trajectory_state = self.trajectory
//         .as_ref()
//         .expect("Trajectory should always be Some at this point since we set it if it was None above")
//         .state_at(self.trajectory_state, self.trajectory_time, self.trajectory_time + self.dt)
//         .expect("Trajectory should always have valid state at current time + dt");
//     self.trajectory_time += self.dt;
    
//     self.prev_body_cmd = Some(target_pose);

//     (global_twist_out, global_accel_out)
// }
