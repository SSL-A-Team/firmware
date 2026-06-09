use crate::motion::params::controller_params::{
    EncLagMode, ENC_LAG_K, ENC_LAG_MODE, ENC_LAG_T_HORIZON, ENC_LAG_T_SLOPE,
    PoseAccelMode, PoseVelMode, POSE_ACCEL_MODE, POSE_VEL_MODE,
    TRAJ_REPLAN_CMD_POS_ANGULAR_RAD, TRAJ_REPLAN_CMD_POS_LINEAR_M,
    TRAJ_REPLAN_CMD_VEL_ANGULAR_RADS, TRAJ_REPLAN_CMD_VEL_LINEAR_MS,
};
use crate::motion::pid::PidController;
use ateam_common_packets::bindings::{
    ParameterCommand, ParameterDataFormat, ParameterName,
};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::robot_model::{KalmanFilterParams, RobotModel, RobotPhysicalParams};
use ateam_controls::{z_rotation_mat, ControlsError, Vector2f, Vector3f, Vector4f, Vector6f, Vector8f};
use ateam_lib_stm32::model::{FirstOrderLag, FirstOrderLagParams};
use core::f32::consts::PI;
use embassy_time::Duration;
use libm::{fabsf, hypotf, remainderf};
use nalgebra::SVector;

pub(crate) const VISION_ACTIVE_TIMEOUT_S: f32 = 0.2;

/// Body-frame setpoints produced by a skill each tick.
/// `body_controller` applies friction compensation and wheel transforms.
#[derive(Copy, Clone, Default)]
pub struct SkillSetpoints {
    pub body_twist: Vector3f,
    pub body_accel: Vector3f,
}

impl SkillSetpoints {
    pub fn zero() -> Self {
        Self {
            body_twist: Vector3f::zeros(),
            body_accel: Vector3f::zeros(),
        }
    }
}

fn pose_cmd_changed(prev: Vector3f, next: Vector3f) -> bool {
    Vector2f::new(next.x - prev.x, next.y - prev.y).norm() > TRAJ_REPLAN_CMD_POS_LINEAR_M
        || remainderf(next.z - prev.z, 2.0 * PI).abs() > TRAJ_REPLAN_CMD_POS_ANGULAR_RAD
}

fn twist_cmd_changed(prev: Vector3f, next: Vector3f) -> bool {
    Vector2f::new(next.x - prev.x, next.y - prev.y).norm() > TRAJ_REPLAN_CMD_VEL_LINEAR_MS
        || fabsf(next.z - prev.z) > TRAJ_REPLAN_CMD_VEL_ANGULAR_RADS
}

/// Frame in which a velocity or acceleration command was issued.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommandFrame {
    Global,
    Local,
}

/// Controller infrastructure passed into each skill on every tick.
///
/// Owns the KF/robot-model, trajectory state, PID, and encoder-lag model.
/// Skills borrow this via `&mut ControlContext` to call planning and
/// tracking helpers. Fields prefixed with "trajectory" and `prev_body_cmd`
/// will migrate into individual skill structs over time.
pub struct ControlContext {
    pub robot_model: RobotModel,
    pub pose_pid_controller: PidController<3>,
    /// Accel (torque) path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_accel_gain: Vector2f,
    /// Velocity setpoint path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_vel_gain: Vector2f,
    /// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
    pub traj_recompute_error: Vector4f,
    /// [LINEAR_VEL_THRESHOLD, LINEAR_ACCEL_THRESHOLD, ANGULAR_VEL_THRESHOLD, ANGULAR_ACCEL_THRESHOLD]
    pub friction_comp_gating: Vector4f,
    pub trajectory: Option<BangBangTraj3D>,
    pub trajectory_time: f32,
    pub trajectory_state: Vector6f,
    pub prev_body_cmd: Option<Vector3f>,
    pub enc_lag: FirstOrderLag<2>,
    pub dt: f32,
    /// Cached KF state estimate — updated each tick before skill dispatch.
    pub state_estimate: Vector6f,
    pub first_vision_received: bool,
    pub time_since_vision_update_s: f32,
    pub wheels_disabled: bool,
}

impl ControlContext {
    pub fn new(dt: f32) -> Self {
        use crate::motion::params::controller_params;

        let enc_lag = FirstOrderLag::new_from_horizon(
            FirstOrderLagParams {
                k: SVector::<f32, 2>::new(ENC_LAG_K[0], ENC_LAG_K[1]),
                t_slope: SVector::<f32, 2>::new(ENC_LAG_T_SLOPE[0], ENC_LAG_T_SLOPE[1]),
            },
            None,
            Duration::from_micros((dt * 1e6) as u64),
            ENC_LAG_T_HORIZON,
        );

        defmt::info!(
            "enc lag: n_steps={}, cmd amplification @ 1 m/s: {}x, @ 5 m/s: {}x",
            enc_lag.n_steps(),
            enc_lag.cmd_amplification(1.0, 0),
            enc_lag.cmd_amplification(5.0, 0),
        );

        Self {
            robot_model: RobotModel::new(
                dt,
                KalmanFilterParams::default(),
                RobotPhysicalParams::default(),
            )
            .expect("Failed to create RobotModel, check that parameters are valid"),
            pose_pid_controller: PidController::<3>::from_gains_matrix_with_anti_jitter(
                &controller_params::pose_pid_gains(),
                Some(controller_params::POSE_PID_ANTI_JITTER_THRESH),
            ),
            pose_accel_gain: controller_params::POSE_ACCEL_GAIN,
            pose_vel_gain: controller_params::POSE_VEL_GAIN,
            traj_recompute_error: controller_params::TRAJ_RECOMPUTE_ERROR,
            friction_comp_gating: controller_params::FRICTION_COMP_GATING,
            trajectory: None,
            trajectory_state: Vector6f::zeros(),
            trajectory_time: 0.0,
            prev_body_cmd: None,
            enc_lag,
            dt,
            state_estimate: Vector6f::zeros(),
            first_vision_received: false,
            time_since_vision_update_s: VISION_ACTIVE_TIMEOUT_S + 1.0,
            wheels_disabled: true,
        }
    }

    pub fn vision_active(&self) -> bool {
        self.first_vision_received && self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S
    }

    pub fn reset(&mut self) {
        self.robot_model.reset();
        self.pose_pid_controller.reset();
        self.trajectory = None;
        self.trajectory_state = Vector6f::default();
        self.trajectory_time = 0.0;
        self.prev_body_cmd = None;
        self.first_vision_received = false;
        self.time_since_vision_update_s = VISION_ACTIVE_TIMEOUT_S + 1.0;
        self.enc_lag.reset();
        self.wheels_disabled = true;
    }

    /// Run KF vision handling, measurement construction, and state update.
    /// Returns the pre-update state prediction used for telemetry.
    pub fn update_state_estimate(
        &mut self,
        vision_pose_meas: Vector3f,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
    ) -> Result<Vector6f, ControlsError> {
        if vision_update {
            if !self.first_vision_received {
                self.robot_model.kf_set_pose(vision_pose_meas);
                self.first_vision_received = true;
                self.prev_body_cmd = None;
                self.pose_pid_controller.reset();
            } else if self.time_since_vision_update_s > VISION_ACTIVE_TIMEOUT_S {
                if self.trajectory.is_some() {
                    let traj_state_pose: Vector3f =
                        self.trajectory_state.fixed_rows::<3>(0).into();
                    let linear_drift = hypotf(
                        traj_state_pose.x - vision_pose_meas.x,
                        traj_state_pose.y - vision_pose_meas.y,
                    );
                    let angular_drift =
                        fabsf(remainderf(traj_state_pose.z - vision_pose_meas.z, 2.0 * PI));
                    if linear_drift > self.traj_recompute_error[0]
                        || angular_drift > self.traj_recompute_error[1]
                    {
                        self.robot_model.kf_set_pose(vision_pose_meas);
                        self.prev_body_cmd = None;
                    }
                }
            }
            self.time_since_vision_update_s = 0.0;
        } else if self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S {
            self.time_since_vision_update_s += self.dt;
        }

        let state_prediction = self.robot_model.get_state();

        let measurement: Vector8f = if matches!(
            ENC_LAG_MODE,
            EncLagMode::KfCorrectionOnly | EncLagMode::Full
        ) {
            let lag_state = self.enc_lag.state();
            let lag_body = Vector3f::new(lag_state.x, lag_state.y, imu_gyro_theta_meas);
            let lag_wheel =
                self.robot_model.transform_twist2wheel(state_prediction[2]) * lag_body;
            nalgebra::vector![
                vision_pose_meas.x,
                vision_pose_meas.y,
                vision_pose_meas.z,
                lag_wheel.x,
                lag_wheel.y,
                lag_wheel.z,
                lag_wheel.w,
                imu_gyro_theta_meas,
            ]
        } else {
            nalgebra::vector![
                vision_pose_meas.x,
                vision_pose_meas.y,
                vision_pose_meas.z,
                wheel_vel_meas.x,
                wheel_vel_meas.y,
                wheel_vel_meas.z,
                wheel_vel_meas.w,
                imu_gyro_theta_meas,
            ]
        };

        self.robot_model.kf_update(measurement, !vision_update, false, false)?;
        self.state_estimate = self.robot_model.get_state();

        Ok(state_prediction)
    }

    /// Compute global-frame friction force from current state and commanded body twist/accel.
    pub fn compute_friction(&self, body_twist: Vector3f, body_accel: Vector3f) -> Vector3f {
        let state_estimate = self.state_estimate;
        let theta = state_estimate.z;
        let r_glob_to_loc = z_rotation_mat(-theta);
        let r_loc_to_glob = z_rotation_mat(theta);

        let est_twist_global: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let est_twist_local: Vector3f = r_glob_to_loc * est_twist_global;
        let tgt_twist_local: Vector3f = r_glob_to_loc * body_twist;
        let cmd_accel_local: Vector3f = r_glob_to_loc * body_accel;

        let lin_vel_mag = hypotf(est_twist_local.x, est_twist_local.y);
        let lin_accel_mag = hypotf(cmd_accel_local.x, cmd_accel_local.y);
        let ang_vel_mag = fabsf(est_twist_local.z);
        let ang_accel_mag = fabsf(cmd_accel_local.z);

        let linear_comp_on = lin_accel_mag >= self.friction_comp_gating[1]
            || lin_vel_mag >= self.friction_comp_gating[0];
        let angular_comp_on = ang_accel_mag >= self.friction_comp_gating[3]
            || ang_vel_mag >= self.friction_comp_gating[2];

        let fric_twist_local = Vector3f::new(
            if linear_comp_on { tgt_twist_local.x } else { 0.0 },
            if linear_comp_on { tgt_twist_local.y } else { 0.0 },
            if angular_comp_on { tgt_twist_local.z } else { 0.0 },
        );

        let friction_force_local = self.robot_model.compute_friction_force(fric_twist_local);
        r_loc_to_glob * friction_force_local
    }

    // -----------------------------------------------------------------------
    // Parameter read/write (data operations only — reset is caller's concern)
    // -----------------------------------------------------------------------

    pub fn expected_format(name: ParameterName::Type) -> Option<ParameterDataFormat::Type> {
        match name {
            ParameterName::KF_PROCESS_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MEASUREMENT_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MAX_STATE => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_WHEEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_INERTIA => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_MOTOR_MODEL => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_FRICTION_MODEL => Some(ParameterDataFormat::VEC6_F32),
            ParameterName::FRICTION_COMP_GATING => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_CONTROL_GAIN => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRAJ_RECOMPUTE_ERROR => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_PIDII_LINEAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_ANGULAR => Some(ParameterDataFormat::VEC5_F32),
            _ => None,
        }
    }

    pub fn read_param(&self, name: ParameterName::Type, reply: &mut ParameterCommand) {
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
                reply.data.vec6_f32 = [
                    phys.coulomb_friction_coefficient_linear_x,
                    phys.coulomb_friction_coefficient_linear_y,
                    phys.coulomb_friction_coefficient_angular,
                    phys.viscous_friction_coefficient_linear_x,
                    phys.viscous_friction_coefficient_linear_y,
                    phys.viscous_friction_coefficient_angular,
                ];
            }
            ParameterName::FRICTION_COMP_GATING => {
                reply.data.vec4_f32 = self.friction_comp_gating.into();
            }
            ParameterName::POSE_CONTROL_GAIN => {
                reply.data.vec2_f32 = self.pose_accel_gain.into();
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                reply.data.vec4_f32 = self.traj_recompute_error.into();
            }
            ParameterName::POSE_FB_PIDII_LINEAR | ParameterName::POSE_FB_PIDII_ANGULAR => {
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
            _ => unreachable!(),
        }
    }

    pub fn write_param(&mut self, cmd: &ParameterCommand) {
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
                let v = unsafe { cmd.data.vec6_f32 };
                let mut p = self.robot_model.physical_params;
                p.coulomb_friction_coefficient_linear_x = v[0];
                p.coulomb_friction_coefficient_linear_y = v[1];
                p.coulomb_friction_coefficient_angular = v[2];
                p.viscous_friction_coefficient_linear_x = v[3];
                p.viscous_friction_coefficient_linear_y = v[4];
                p.viscous_friction_coefficient_angular = v[5];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::FRICTION_COMP_GATING => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.friction_comp_gating = Vector4f::new(v[0], v[1], v[2], v[3]);
            }
            ParameterName::POSE_CONTROL_GAIN => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.pose_accel_gain = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.traj_recompute_error = Vector4f::new(v[0], v[1], v[2], v[3]);
            }
            ParameterName::POSE_FB_PIDII_LINEAR | ParameterName::POSE_FB_PIDII_ANGULAR => {
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
            _ => unreachable!(),
        }
    }

    // -----------------------------------------------------------------------
    // Control policies
    // -----------------------------------------------------------------------

    /// Produce body-frame (twist, accel) setpoints for a global-position target.
    ///
    /// Disables wheels and resets state if vision is not active.
    pub fn pose_control_policy(
        &mut self,
        target_pose: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        if !self.vision_active() {
            if self.first_vision_received {
                self.reset();
            }
            self.wheels_disabled = true;
            return Ok((Vector3f::zeros(), Vector3f::zeros()));
        }

        let state_estimate = self.state_estimate;
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let cmd_changed = self
            .prev_body_cmd
            .map_or(true, |cmd| pose_cmd_changed(cmd, target_pose));

        let traj = match self.trajectory {
            Some(traj) if !cmd_changed => {
                if self.tracking_error_exceeded(pose_estimate, twist_estimate) {
                    self.plan_pose_trajectory(state_estimate, target_pose, traj_params)?
                } else {
                    traj
                }
            }
            Some(_) => {
                let seed_state = if self.tracking_error_exceeded(pose_estimate, twist_estimate) {
                    state_estimate
                } else {
                    self.trajectory_state
                };
                self.plan_pose_trajectory(seed_state, target_pose, traj_params)?
            }
            None => self.plan_pose_trajectory(state_estimate, target_pose, traj_params)?,
        };

        let result = self.track_trajectory(state_estimate, traj)?;

        if cmd_changed {
            self.prev_body_cmd = Some(target_pose);
        }

        Ok(result)
    }

    /// Produce body-frame (twist, accel) setpoints for a velocity target.
    pub fn twist_control_policy(
        &mut self,
        target_twist: Vector3f,
        frame: CommandFrame,
        traj_params: TrajectoryParams,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        let state_estimate = self.state_estimate;
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let cmd_changed = self
            .prev_body_cmd
            .map_or(true, |cmd| twist_cmd_changed(cmd, target_twist));

        let global_target_twist = match frame {
            CommandFrame::Global => target_twist,
            CommandFrame::Local => z_rotation_mat(state_estimate.z) * target_twist,
        };

        let traj = match self.trajectory {
            Some(traj) if !cmd_changed => {
                if self.tracking_error_exceeded(pose_estimate, twist_estimate) {
                    self.plan_twist_trajectory(state_estimate, global_target_twist, traj_params)?
                } else {
                    traj
                }
            }
            Some(_) => {
                let seed_state = if self.tracking_error_exceeded(pose_estimate, twist_estimate) {
                    state_estimate
                } else {
                    self.trajectory_state
                };
                self.plan_twist_trajectory(seed_state, global_target_twist, traj_params)?
            }
            None => {
                self.plan_twist_trajectory(state_estimate, global_target_twist, traj_params)?
            }
        };

        let result = self.track_trajectory(state_estimate, traj)?;

        if cmd_changed {
            self.prev_body_cmd = Some(target_twist);
        }

        Ok(result)
    }

    /// Produce body-frame (twist, accel) setpoints for a direct acceleration command.
    pub fn accel_control_policy(
        &mut self,
        target_accel: Vector3f,
        frame: CommandFrame,
    ) -> (Vector3f, Vector3f) {
        let state_estimate = self.state_estimate;
        let target_accel = match frame {
            CommandFrame::Global => target_accel,
            CommandFrame::Local => z_rotation_mat(state_estimate.z) * target_accel,
        };
        let next_state = self.robot_model.a * state_estimate + self.robot_model.b * target_accel;
        let twist_out = next_state.fixed_rows::<3>(3).into();
        (twist_out, target_accel)
    }

    fn plan_twist_trajectory(
        &mut self,
        seed_state: Vector6f,
        target_twist: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<BangBangTraj3D, ControlsError> {
        let seed_twist: Vector3f = seed_state.fixed_rows::<3>(3).into();
        self.trajectory_state = seed_state;
        self.trajectory_time = 0.0;
        let traj = BangBangTraj3D::from_target_twist(seed_twist, target_twist, traj_params)?;
        self.trajectory = Some(traj);
        Ok(traj)
    }

    fn plan_pose_trajectory(
        &mut self,
        seed_state: Vector6f,
        target_pose: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<BangBangTraj3D, ControlsError> {
        self.trajectory_state = seed_state;
        self.trajectory_time = 0.0;
        let traj = BangBangTraj3D::from_target_pose(seed_state, target_pose, traj_params)?;
        self.trajectory = Some(traj);
        Ok(traj)
    }

    fn tracking_error_exceeded(
        &self,
        pose_estimate: Vector3f,
        twist_estimate: Vector3f,
    ) -> bool {
        let traj_state_pose: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
        let traj_state_twist: Vector3f = self.trajectory_state.fixed_rows::<3>(3).into();
        let linear_pose_error = hypotf(
            traj_state_pose.x - pose_estimate.x,
            traj_state_pose.y - pose_estimate.y,
        );
        let angular_pose_error =
            fabsf(remainderf(traj_state_pose.z - pose_estimate.z, 2.0 * PI));
        let linear_twist_error = hypotf(
            traj_state_twist.x - twist_estimate.x,
            traj_state_twist.y - twist_estimate.y,
        );
        let angular_twist_error = fabsf(traj_state_twist.z - twist_estimate.z);
        linear_pose_error > self.traj_recompute_error[0]
            || angular_pose_error > self.traj_recompute_error[1]
            || linear_twist_error > self.traj_recompute_error[2]
            || angular_twist_error > self.traj_recompute_error[3]
    }

    fn track_trajectory(
        &mut self,
        state_estimate: Vector6f,
        traj: BangBangTraj3D,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let mut traj_curstep_tgt_pos: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
        let traj_curstep_tgt_vel: Vector3f = self.trajectory_state.fixed_rows::<3>(3).into();
        traj_curstep_tgt_pos.z =
            pose_estimate.z + remainderf(traj_curstep_tgt_pos.z - pose_estimate.z, 2.0 * PI);
        let twist_error = traj_curstep_tgt_vel - twist_estimate;
        let pos_pid_feedback = self.pose_pid_controller.calculate_with_derivative(
            &traj_curstep_tgt_pos,
            &pose_estimate,
            &twist_error,
            self.dt,
        );

        let accel_out: Vector3f = {
            let accel_ff_term = if matches!(
                POSE_ACCEL_MODE,
                PoseAccelMode::FeedforwardOnly | PoseAccelMode::Full
            ) {
                self.pose_accel_gain[0] * traj.accel_at(self.trajectory_time)?
            } else {
                Vector3f::zeros()
            };

            let accel_fb_term = if matches!(
                POSE_ACCEL_MODE,
                PoseAccelMode::FeedbackOnly | PoseAccelMode::Full
            ) {
                self.pose_accel_gain[1] * pos_pid_feedback
            } else {
                Vector3f::zeros()
            };

            accel_ff_term + accel_fb_term
        };

        let twist_out: Vector3f = {
            let vel_ff_term: Vector3f = if matches!(
                POSE_VEL_MODE,
                PoseVelMode::FeedforwardOnly | PoseVelMode::Full
            ) {
                self.pose_vel_gain[0] * traj_curstep_tgt_vel
            } else {
                (state_estimate.fixed_rows::<3>(3) + accel_out * self.dt).into()
            };

            let vel_fb_term =
                if matches!(POSE_VEL_MODE, PoseVelMode::FeedbackOnly | PoseVelMode::Full) {
                    self.pose_vel_gain[1] * pos_pid_feedback * self.dt
                } else {
                    Vector3f::zeros()
                };

            vel_ff_term + vel_fb_term
        };

        self.trajectory_state = traj.state_at(
            self.trajectory_state,
            self.trajectory_time,
            self.trajectory_time + self.dt,
        )?;
        self.trajectory_time += self.dt;

        Ok((twist_out, accel_out))
    }
}
