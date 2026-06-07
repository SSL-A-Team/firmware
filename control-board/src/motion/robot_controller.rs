use crate::motion::params::controller_params::{
    EncLagMode, PoseAccelMode, PoseVelMode, ENC_LAG_K, ENC_LAG_MODE, ENC_LAG_T_HORIZON,
    ENC_LAG_T_SLOPE, POSE_ACCEL_MODE, POSE_VEL_MODE, TRAJ_REPLAN_CMD_ANGLE_RAD,
    TRAJ_REPLAN_CMD_POS_DIST_M,
};
use crate::motion::pid::PidController;
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{
    BasicControl, BodyControlExtendedTelemetry, BodyControlTelemetry,
    ExtendedGlobalAccelerationTelemetry, ExtendedGlobalPositionTelemetry,
    ExtendedGlobalVelocityTelemetry, ExtendedLocalAccelerationTelemetry,
    ExtendedLocalVelocityTelemetry, ParameterCommandCode::*, ParameterDataFormat, ParameterName,
};
use ateam_common_packets::radio::{SkillCommand, SkillExtendedTelemetry};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::{
    z_rotation_mat, ControlsError, Vector2f, Vector3f, Vector4f, Vector6f, Vector8f,
};
use ateam_lib_stm32::model::{FirstOrderLag, FirstOrderLagParams};
use core::f32::consts::PI;
use embassy_time::{Duration, Instant};
use libm::{fabsf, hypotf, remainderf};
use nalgebra::vector;
use nalgebra::SVector;

use ateam_common_packets::bindings::ParameterCommand;

/// Time (seconds) without a vision update after which vision is considered "inactive".
const VISION_ACTIVE_TIMEOUT_S: f32 = 0.2;

pub struct BodyController {
    pub robot_model: RobotModel,
    pub pose_pid_controller: PidController<3>,
    /// Accel (torque) path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_accel_gain: Vector2f,
    /// Velocity setpoint path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_vel_gain: Vector2f,
    /// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR] thresholds for when to recompute the trajectory
    pub traj_recompute_error: Vector4f,
    /// [LINEAR_VEL_THRESHOLD, LINEAR_ACCEL_THRESHOLD, ANGULAR_VEL_THRESHOLD, ANGULAR_ACCEL_THRESHOLD]
    pub friction_comp_gating: Vector4f,
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
    pub enc_lag: FirstOrderLag<2>,
    pub wheels_disabled: bool,
}

fn pose_cmd_changed(prev: Vector3f, next: Vector3f) -> bool {
    Vector2f::new(next.x - prev.x, next.y - prev.y).norm() > TRAJ_REPLAN_CMD_POS_DIST_M
        || remainderf(next.z - prev.z, 2.0 * PI).abs() > TRAJ_REPLAN_CMD_ANGLE_RAD
}

fn twist_cmd_changed(prev: Vector3f, next: Vector3f) -> bool {
    Vector2f::new(next.x - prev.x, next.y - prev.y).norm() > 0.01 || fabsf(next.z - prev.z) > 0.1
}

impl BodyController {
    pub fn new(dt: f32) -> BodyController {
        use crate::motion::params::controller_params;
        use ateam_controls::robot_model::{KalmanFilterParams, RobotPhysicalParams};

        let controller = BodyController {
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
            body_twist_out: Vector3f::default(),
            body_accel_out: Vector3f::default(),
            body_accel_out_fric_comp: Vector3f::default(),
            wheel_vel_out: Vector4f::default(),
            wheel_torque_out: Vector4f::default(),
            telemetry: Default::default(),
            debug_telemetry: Default::default(),
            dt,
            first_vision_received: false,
            time_since_vision_update_s: VISION_ACTIVE_TIMEOUT_S + 1.0, // Set to higher than timeout
            enc_lag: FirstOrderLag::new_from_horizon(
                FirstOrderLagParams {
                    k: SVector::<f32, 2>::new(ENC_LAG_K[0], ENC_LAG_K[1]),
                    t_slope: SVector::<f32, 2>::new(ENC_LAG_T_SLOPE[0], ENC_LAG_T_SLOPE[1]),
                },
                None,
                Duration::from_micros((dt * 1e6) as u64),
                ENC_LAG_T_HORIZON,
            ),
            wheels_disabled: true,
        };

        defmt::info!(
            "enc lag: n_steps={}, cmd amplification @ 1 m/s: {}x, @ 5 m/s: {}x",
            controller.enc_lag.n_steps(),
            controller.enc_lag.cmd_amplification(1.0, 0),
            controller.enc_lag.cmd_amplification(5.0, 0),
        );

        controller
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
        self.debug_telemetry = Default::default();
        self.first_vision_received = false;
        self.time_since_vision_update_s = VISION_ACTIVE_TIMEOUT_S + 1.0; // Set to higher than timeout
        self.enc_lag.reset();
        self.wheels_disabled = true;
    }

    pub fn vision_active(&self) -> bool {
        self.first_vision_received && self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S
    }

    pub fn wheels_disabled(&self) -> bool {
        self.wheels_disabled
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

        // Default to enabled; policies that need to lock out motors (e.g.,
        // pose mode without vision) will set this back to true below.
        self.wheels_disabled = false;

        // Working in global frame, unless variable is specified as local

        if vision_update {
            if !self.first_vision_received {
                // First vision sample since controller init or reset. Snap
                // the KF to it and force a replan on the next tracker call:
                // any in-flight trajectory was built against a default-zero
                // KF pose
                self.robot_model.kf_set_pose(vision_pose_meas);
                self.first_vision_received = true;
                self.prev_body_cmd = None;
                self.pose_pid_controller.reset();
            } else if self.time_since_vision_update_s > VISION_ACTIVE_TIMEOUT_S {
                // Vision returning after a dropout: if the integrated trajectory
                // pose has drifted from the new vision pose beyond the replan
                // thresholds, snap the KF to vision and force a replan on the
                // next tracker call.
                if self.trajectory.is_some() {
                    let traj_state_pose: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
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
        } else {
            if self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S {
                self.time_since_vision_update_s += self.dt; // Only add when less than timeout
            }
        }

        let state_prediction = self.robot_model.get_state();

        // When KfCorrectionOnly or Full the raw encoder rows are replaced with the lag
        // model's prediction of what the encoders should be showing this tick
        // (driven by last tick's command), so the KF update compares like-for-like.
        let measurement: Vector8f = if matches!(
            ENC_LAG_MODE,
            EncLagMode::KfCorrectionOnly | EncLagMode::Full
        ) {
            let lag_state = self.enc_lag.state();
            let lag_body = Vector3f::new(lag_state.x, lag_state.y, imu_gyro_theta_meas);
            let lag_wheel = self.robot_model.transform_twist2wheel(state_prediction[2]) * lag_body;
            vector![
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
            vector![
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

        self.robot_model
            .kf_update(measurement, !vision_update, false, false)?;
        let state_estimate = self.robot_model.get_state();

        let t_after_kf_update = Instant::now();

        let body_twist_out;
        let body_accel_out;
        let skill_telem;
        match last_command.get_skill_command() {
            SkillCommand::GlobalPosition(gpos_cmd) => {
                let default_params = TrajectoryParams::default();
                let traj_params = TrajectoryParams {
                    max_vel_linear: if gpos_cmd.max_linear_vel != 0.0 {
                        gpos_cmd.max_linear_vel
                    } else {
                        default_params.max_vel_linear
                    },
                    max_vel_angular: if gpos_cmd.max_angular_vel != 0.0 {
                        gpos_cmd.max_angular_vel
                    } else {
                        default_params.max_vel_angular
                    },
                    max_accel_linear: if gpos_cmd.max_linear_acc != 0.0 {
                        gpos_cmd.max_linear_acc
                    } else {
                        default_params.max_accel_linear
                    },
                    max_accel_angular: if gpos_cmd.max_angular_acc != 0.0 {
                        gpos_cmd.max_angular_acc
                    } else {
                        default_params.max_accel_angular
                    },
                };
                (body_twist_out, body_accel_out) = self.global_pose_bangbang_pid_control_policy(
                    state_estimate,
                    gpos_cmd.as_vec3f(),
                    traj_params,
                )?;
                skill_telem =
                    SkillExtendedTelemetry::GlobalPosition(ExtendedGlobalPositionTelemetry {
                        cmd_echo: gpos_cmd,
                    });
            }
            SkillCommand::GlobalVelocity(gvel_cmd) => {
                let default_params = TrajectoryParams::default();
                let traj_params = TrajectoryParams {
                    max_vel_linear: default_params.max_vel_linear,
                    max_vel_angular: default_params.max_vel_angular,
                    max_accel_linear: if gvel_cmd.max_linear_acc != 0.0 {
                        gvel_cmd.max_linear_acc
                    } else {
                        default_params.max_accel_linear
                    },
                    max_accel_angular: if gvel_cmd.max_angular_acc != 0.0 {
                        gvel_cmd.max_angular_acc
                    } else {
                        default_params.max_accel_angular
                    },
                };
                (body_twist_out, body_accel_out) = self.global_twist_control_policy(
                    state_estimate,
                    gvel_cmd.as_vec3f(),
                    traj_params,
                )?;
                skill_telem =
                    SkillExtendedTelemetry::GlobalVelocity(ExtendedGlobalVelocityTelemetry {
                        cmd_echo: gvel_cmd,
                    });
            }
            SkillCommand::LocalVelocity(lvel_cmd) => {
                let default_params = TrajectoryParams::default();
                let traj_params = TrajectoryParams {
                    max_vel_linear: default_params.max_vel_linear,
                    max_vel_angular: default_params.max_vel_angular,
                    max_accel_linear: if lvel_cmd.max_linear_acc != 0.0 {
                        lvel_cmd.max_linear_acc
                    } else {
                        default_params.max_accel_linear
                    },
                    max_accel_angular: if lvel_cmd.max_angular_acc != 0.0 {
                        lvel_cmd.max_angular_acc
                    } else {
                        default_params.max_accel_angular
                    },
                };
                (body_twist_out, body_accel_out) = self.local_twist_control_policy(
                    state_estimate,
                    lvel_cmd.as_vec3f(),
                    traj_params,
                )?;
                skill_telem =
                    SkillExtendedTelemetry::LocalVelocity(ExtendedLocalVelocityTelemetry {
                        cmd_echo: lvel_cmd,
                    });
            }
            SkillCommand::GlobalAcceleration(gacc_cmd) => {
                (body_twist_out, body_accel_out) =
                    self.global_accel_control_policy(state_estimate, gacc_cmd.as_vec3f());
                skill_telem = SkillExtendedTelemetry::GlobalAcceleration(
                    ExtendedGlobalAccelerationTelemetry { cmd_echo: gacc_cmd },
                );
            }
            SkillCommand::LocalAcceleration(lacc_cmd) => {
                (body_twist_out, body_accel_out) =
                    self.local_accel_control_policy(state_estimate, lacc_cmd.as_vec3f());
                skill_telem =
                    SkillExtendedTelemetry::LocalAcceleration(ExtendedLocalAccelerationTelemetry {
                        cmd_echo: lacc_cmd,
                    });
            }
            _ => {
                (body_twist_out, body_accel_out) = (Vector3f::zeros(), Vector3f::zeros());
                skill_telem = SkillExtendedTelemetry::Off;
            }
        };

        self.body_twist_out = body_twist_out;
        self.body_accel_out = body_accel_out;

        let friction_force_global = self.compute_friction(state_estimate);
        self.body_accel_out_fric_comp =
            self.body_accel_out - self.robot_model.i_inv * friction_force_global;

        // Calculate wheel commands from body commands.
        // When FeedforwardOnly or Full the XY body velocity is pre-compensated through
        // the lag model inversion before the wheel transform; angular velocity is not lagged.
        let body_xy = SVector::<f32, 2>::new(self.body_twist_out.x, self.body_twist_out.y);
        self.wheel_vel_out =
            if matches!(ENC_LAG_MODE, EncLagMode::FeedforwardOnly | EncLagMode::Full) {
                let compensated_xy = self.enc_lag.invert(&body_xy);
                let compensated_twist =
                    Vector3f::new(compensated_xy.x, compensated_xy.y, self.body_twist_out.z);
                self.robot_model.transform_twist2wheel(state_estimate.z) * compensated_twist
            } else {
                self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_out
            };

        self.wheel_torque_out = self.robot_model.transform_accel2wheel(state_estimate.z)
            * self.body_accel_out_fric_comp;

        if !matches!(ENC_LAG_MODE, EncLagMode::Disabled) {
            self.enc_lag.step(&body_xy);
        }

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

    /// Returns the modeled friction force in the global frame
    ///
    /// Uses friction compensation gating to turn on/off linear and/or angular
    /// friction compensation.
    fn compute_friction(&self, state_estimate: Vector6f) -> Vector3f {
        let theta = state_estimate.z;
        let r_glob_to_loc = z_rotation_mat(-theta);
        let r_loc_to_glob = z_rotation_mat(theta);

        let est_twist_global: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let est_twist_local: Vector3f = r_glob_to_loc * est_twist_global;
        let tgt_twist_local: Vector3f = r_glob_to_loc * self.body_twist_out;
        let cmd_accel_local: Vector3f = r_glob_to_loc * self.body_accel_out;

        let lin_vel_mag = hypotf(est_twist_local.x, est_twist_local.y);
        let lin_accel_mag = hypotf(cmd_accel_local.x, cmd_accel_local.y);
        let ang_vel_mag = fabsf(est_twist_local.z);
        let ang_accel_mag = fabsf(cmd_accel_local.z);

        let lin_vel_thresh = self.friction_comp_gating[0];
        let lin_accel_thresh = self.friction_comp_gating[1];
        let ang_vel_thresh = self.friction_comp_gating[2];
        let ang_accel_thresh = self.friction_comp_gating[3];

        // Gate priority: if the commanded acceleration is above the accel
        // threshold, always compensate (the controller is actively pushing).
        // Otherwise use the velocity threshold (if the robot is moving,
        // compensate to overcome dynamic friction; if the robot is stationary,
        // don't compensate to avoid oscillations from static friction).
        let linear_comp_on = if lin_accel_mag >= lin_accel_thresh {
            true
        } else {
            lin_vel_mag >= lin_vel_thresh
        };
        let angular_comp_on = if ang_accel_mag >= ang_accel_thresh {
            true
        } else {
            ang_vel_mag >= ang_vel_thresh
        };

        let fric_twist_local = Vector3f::new(
            if linear_comp_on {
                tgt_twist_local.x
            } else {
                0.0
            },
            if linear_comp_on {
                tgt_twist_local.y
            } else {
                0.0
            },
            if angular_comp_on {
                tgt_twist_local.z
            } else {
                0.0
            },
        );

        let friction_force_local = self.robot_model.compute_friction_force(fric_twist_local);
        r_loc_to_glob * friction_force_local
    }

    fn global_accel_control_policy(
        &mut self,
        state_estimate: Vector6f,
        target_accel: Vector3f,
    ) -> (Vector3f, Vector3f) {
        let next_state = self.robot_model.a * state_estimate + self.robot_model.b * target_accel;
        let twist_out = next_state.fixed_rows::<3>(3).into();
        let accel_out = target_accel;
        (twist_out, accel_out)
    }

    fn local_accel_control_policy(
        &mut self,
        state_estimate: Vector6f,
        local_target_accel: Vector3f,
    ) -> (Vector3f, Vector3f) {
        let target_accel = z_rotation_mat(state_estimate.z) * local_target_accel;
        self.global_accel_control_policy(state_estimate, target_accel)
    }

    fn local_twist_control_policy(
        &mut self,
        state_estimate: Vector6f,
        local_target_twist: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        let target_twist = z_rotation_mat(state_estimate.z) * local_target_twist;
        self.global_twist_control_policy(state_estimate, target_twist, traj_params)
    }

    fn plan_twist_trajectory(
        &mut self,
        state_estimate: Vector6f,
        target_twist: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<BangBangTraj3D, ControlsError> {
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        self.trajectory_state = state_estimate;
        self.trajectory_time = 0.0;
        let traj = BangBangTraj3D::from_target_twist(twist_estimate, target_twist, traj_params)?;
        self.trajectory = Some(traj);
        Ok(traj)
    }

    fn global_twist_control_policy(
        &mut self,
        state_estimate: Vector6f,
        target_twist: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let cmd_changed = self
            .prev_body_cmd
            .map_or(true, |cmd| twist_cmd_changed(cmd, target_twist));

        // Replan trajectory if we don't have one, the command changed, or
        // tracking errors (pose against integrated traj pose, plus twist)
        // have grown too large.
        let traj = match self.trajectory {
            Some(traj) if !cmd_changed => {
                if self.tracking_error_exceeded(pose_estimate, twist_estimate) {
                    self.plan_twist_trajectory(state_estimate, target_twist, traj_params)?
                } else {
                    traj
                }
            }
            _ => self.plan_twist_trajectory(state_estimate, target_twist, traj_params)?,
        };

        let result = self.track_trajectory(state_estimate, traj)?;

        // Don't latch a new command if it doesn't meet the change threshold.
        // Otherwise a small twist command ramp could never trigger a replan.
        if cmd_changed {
            self.prev_body_cmd = Some(target_twist);
        }

        Ok(result)
    }

    fn plan_pose_trajectory(
        &mut self,
        state_estimate: Vector6f,
        target_pose: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<BangBangTraj3D, ControlsError> {
        self.trajectory_state = state_estimate;
        self.trajectory_time = 0.0;
        let traj = BangBangTraj3D::from_target_pose(state_estimate, target_pose, traj_params)?;
        self.trajectory = Some(traj);
        Ok(traj)
    }

    fn global_pose_bangbang_pid_control_policy(
        &mut self,
        state_estimate: Vector6f,
        target_pose: Vector3f,
        traj_params: TrajectoryParams,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        // Vision required for pose control. On the transition into the
        // no-vision state, fully reset the controller so we start clean once
        // vision returns. While in this state we signal `wheels_disabled`
        if !self.vision_active() {
            if self.first_vision_received {
                // Transition: we had vision and now we don't. Wipe trajectory,
                // PID, and KF state; `reset()` also clears `first_vision_received`
                // so the next vision sample is treated as the first one.
                self.reset();
            }
            self.wheels_disabled = true;
            return Ok((Vector3f::zeros(), Vector3f::zeros()));
        }

        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let cmd_changed = self
            .prev_body_cmd
            .map_or(true, |cmd| pose_cmd_changed(cmd, target_pose));

        // Replan trajectory if we don't have one, the command changed, or
        // tracking errors have grown too large.
        let traj = match self.trajectory {
            Some(traj) if !cmd_changed => {
                if self.tracking_error_exceeded(pose_estimate, twist_estimate) {
                    self.plan_pose_trajectory(state_estimate, target_pose, traj_params)?
                } else {
                    traj
                }
            }
            _ => self.plan_pose_trajectory(state_estimate, target_pose, traj_params)?,
        };

        let result = self.track_trajectory(state_estimate, traj)?;

        // Don't latch a new command if it doesn't meet the change threshold.
        // Otherwise a small pose command ramp could never trigger a replan.
        if cmd_changed {
            self.prev_body_cmd = Some(target_pose);
        }

        Ok(result)
    }

    /// Returns true if the integrated trajectory state has drifted from the
    /// state estimate beyond `traj_recompute_error` thresholds     
    fn tracking_error_exceeded(&self, pose_estimate: Vector3f, twist_estimate: Vector3f) -> bool {
        let traj_state_pose: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
        let traj_state_twist: Vector3f = self.trajectory_state.fixed_rows::<3>(3).into();
        let linear_pose_error = hypotf(
            traj_state_pose.x - pose_estimate.x,
            traj_state_pose.y - pose_estimate.y,
        );
        let angular_pose_error = fabsf(remainderf(traj_state_pose.z - pose_estimate.z, 2.0 * PI));
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

    /// Shared trajectory tracker used by both pose and twist control policies.
    ///
    /// Pre-conditions: `self.trajectory == Some(traj)`, and
    /// `self.trajectory_state` / `self.trajectory_time` correspond to the
    /// current point on `traj` that should be tracked this tick.
    fn track_trajectory(
        &mut self,
        state_estimate: Vector6f,
        traj: BangBangTraj3D,
    ) -> Result<(Vector3f, Vector3f), ControlsError> {
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        // PID feedback — computed unconditionally to keep integrator state current.
        let mut traj_curstep_tgt_pos: Vector3f = self.trajectory_state.fixed_rows::<3>(0).into();
        let traj_curstep_tgt_vel: Vector3f = self.trajectory_state.fixed_rows::<3>(3).into();
        // Wrap target theta so that (target.z - pose_estimate.z) lies in [-π, π]
        traj_curstep_tgt_pos.z =
            pose_estimate.z + remainderf(traj_curstep_tgt_pos.z - pose_estimate.z, 2.0 * PI);
        let twist_error = traj_curstep_tgt_vel - twist_estimate;
        let pos_pid_feedback = self.pose_pid_controller.calculate_with_derivative(
            &traj_curstep_tgt_pos,
            &pose_estimate,
            &twist_error,
            self.dt,
        );

        // Accel output (torque path) — gated by POSE_ACCEL_MODE.
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

        // Velocity setpoint (wheel vel path) — gated by POSE_VEL_MODE.
        // FeedforwardOnly and Full use trajectory velocity as the base;
        // Disabled and FeedbackOnly derive velocity by integrating accel_out.
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

        // Step trajectory forward
        self.trajectory_state = traj.state_at(
            self.trajectory_state,
            self.trajectory_time,
            self.trajectory_time + self.dt,
        )?;
        self.trajectory_time += self.dt;

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
            ParameterName::PHYS_FRICTION_MODEL => Some(ParameterDataFormat::VEC6_F32),
            ParameterName::FRICTION_COMP_GATING => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_CONTROL_GAIN => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRAJ_RECOMPUTE_ERROR => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_PIDII_LINEAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_ANGULAR => Some(ParameterDataFormat::VEC5_F32),
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
                reply.data.vec4_f32 = [
                    self.friction_comp_gating[0],
                    self.friction_comp_gating[1],
                    self.friction_comp_gating[2],
                    self.friction_comp_gating[3],
                ];
            }
            ParameterName::POSE_CONTROL_GAIN => {
                reply.data.vec2_f32 = [self.pose_accel_gain[0], self.pose_accel_gain[1]];
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                reply.data.vec4_f32 = [
                    self.traj_recompute_error[0],
                    self.traj_recompute_error[1],
                    self.traj_recompute_error[2],
                    self.traj_recompute_error[3],
                ];
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
                defmt::warn!(
                    "unexpected parameter name {}, cannot apply command",
                    param_cmd.parameter_name
                );
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
