use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackingDivergenceState};
use crate::motion::maneuvers::ManeuverManager;
use crate::motion::params::controller_params::{
    EncLagMode, BODY_ACCEL_CLAMP_ANGULAR, BODY_ACCEL_CLAMP_LINEAR, BODY_VEL_CLAMP_ANGULAR,
    BODY_VEL_CLAMP_LINEAR, ENC_LAG_MODE, STOP_STATE_LINEAR_SPEED_LIMIT,
};
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::{
    BasicControl, BodyControlCommand, BodyControlExtendedTelemetry,
    BodyControlManeuverExtendedTelemetry, BodyControlTelemetry, ParameterCommand,
    ParameterCommandCode, ParameterName,
};
use ateam_common_packets::bitfields::BodyControlExtTelemetryFlags;
use ateam_controls::trajectory::Trajectory;
use ateam_controls::{ControlsError, Vector3f, Vector4f};
use embassy_time::Instant;
use nalgebra::SVector;

// ---------------------------------------------------------------------------
// BodyController
// ---------------------------------------------------------------------------

pub struct BodyController {
    pub control_context: ControlContext,
    pub maneuver_manager: ManeuverManager,
    pub body_twist_out: Vector3f,
    pub body_accel_out: Vector3f,
    pub body_accel_out_fric_comp: Vector3f,
    pub wheel_vel_out: Vector4f,
    pub wheel_torque_out: Vector4f,
    pub telemetry: BodyControlTelemetry,
    pub debug_telemetry: BodyControlExtendedTelemetry,
}

impl BodyController {
    pub fn new(dt: f32) -> BodyController {
        BodyController {
            control_context: ControlContext::new(dt),
            maneuver_manager: ManeuverManager::new(),
            body_twist_out: Vector3f::default(),
            body_accel_out: Vector3f::default(),
            body_accel_out_fric_comp: Vector3f::default(),
            wheel_vel_out: Vector4f::default(),
            wheel_torque_out: Vector4f::default(),
            telemetry: Default::default(),
            debug_telemetry: Default::default(),
        }
    }

    pub fn reset(&mut self) {
        self.control_context.reset();
        self.maneuver_manager.reset();
        self.body_twist_out = Vector3f::default();
        self.body_accel_out = Vector3f::default();
        self.body_accel_out_fric_comp = Vector3f::default();
        self.wheel_vel_out = Vector4f::default();
        self.wheel_torque_out = Vector4f::default();
        self.telemetry = Default::default();
        self.debug_telemetry = Default::default();
    }

    pub fn vision_active(&self) -> bool {
        self.control_context.vision_active()
    }

    pub fn wheels_disabled(&self) -> bool {
        self.control_context.wheels_disabled
    }

    /// True while recovering from a trajectory divergence (e.g. a collision); the
    /// control task should command the active brake. Reset happens automatically
    /// once the wheels stop.
    pub fn tracking_divergence_recovery_active(&self) -> bool {
        self.control_context.tracking_divergence_state == TrackingDivergenceState::Recovering
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
    ) -> Result<(bool, bool), ControlsError> {
        let t_start = Instant::now();

        self.control_context.wheels_disabled = false;

        let state_prediction = self.control_context.update_state_estimate(
            vision_pose_meas,
            vision_update,
            wheel_vel_meas,
            imu_gyro_theta_meas,
        )?;

        let t_after_kf_update = Instant::now();

        // While the robot is halted (SSL HALT game state) or recovering from a
        // trajectory divergence (e.g. a collision), the control task ignores the
        // maneuver outputs and commands the active brake instead. Progressing the
        // maneuver anyway would keep advancing its trajectory clock and publish
        // telemetry (trajectory pose/vel, commanded twist/accel, maneuver state)
        // implying motion the robot is not performing, which is confusing to
        // observe. Hold the maneuver in a reset state and emit zero setpoints;
        // when the state clears, the maneuver re-enters and replans from the
        // current state estimate.
        //
        // The divergence-recovery state is read before running its state machine
        // below, so it reflects the decision made on the previous tick.
        let hold_maneuvers = last_command.flags.game_state_in_halt()
            || self.control_context.tracking_divergence_state
                == TrackingDivergenceState::Recovering
            || matches!(last_command.cmd, BodyControlCommand::Off | BodyControlCommand::EstopBrake);

        let (setpoints, maneuver_telem) = if hold_maneuvers {
            self.maneuver_manager.reset();
            self.control_context.reset_trajectory();
            (ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off)
        } else {
            self.maneuver_manager
                .tick(last_command, &mut self.control_context)?
        };

        let disabled = self.control_context.wheels_disabled;
        // Trajectory-divergence recovery: a large unexpected tracking error (e.g.
        // a collision) trips into a braking recovery; the controller resets only
        // after braking finishes, then tracking replans from the fresh estimate.
        self.control_context
            .update_tracking_divergence_recovery(wheel_vel_meas);
        if !disabled {
            self.control_context.wheels_disabled = false;
        }

        self.body_twist_out = setpoints.body_twist;
        self.body_accel_out = setpoints.body_accel;

        // SSL stop state: clamp linear speed after control policy output so feedback
        // loops cannot overshoot to recover trajectory error.
        if last_command.flags.game_state_in_stop() {
            let linear_speed = self.body_twist_out.xy().norm();
            if linear_speed > STOP_STATE_LINEAR_SPEED_LIMIT {
                let scale = STOP_STATE_LINEAR_SPEED_LIMIT / linear_speed;
                self.body_twist_out.x *= scale;
                self.body_twist_out.y *= scale;
            }
        }

        // Clamp body-level velocity before converting to wheel velocity setpoints.
        let twist_clamped_x = self
            .body_twist_out
            .x
            .clamp(-BODY_VEL_CLAMP_LINEAR, BODY_VEL_CLAMP_LINEAR);
        let twist_clamped_y = self
            .body_twist_out
            .y
            .clamp(-BODY_VEL_CLAMP_LINEAR, BODY_VEL_CLAMP_LINEAR);
        let twist_clamped_z = self
            .body_twist_out
            .z
            .clamp(-BODY_VEL_CLAMP_ANGULAR, BODY_VEL_CLAMP_ANGULAR);
        let vel_clamped = twist_clamped_x != self.body_twist_out.x
            || twist_clamped_y != self.body_twist_out.y
            || twist_clamped_z != self.body_twist_out.z;
        self.body_twist_out = Vector3f::new(twist_clamped_x, twist_clamped_y, twist_clamped_z);

        let friction_force_global = self
            .control_context
            .compute_friction(self.body_twist_out, self.body_accel_out);
        let body_accel_fric_comp =
            self.body_accel_out - self.control_context.robot_model.i_inv * friction_force_global;

        // Clamp body-level acceleration before converting to wheel torques.
        let clamped_x = body_accel_fric_comp
            .x
            .clamp(-BODY_ACCEL_CLAMP_LINEAR, BODY_ACCEL_CLAMP_LINEAR);
        let clamped_y = body_accel_fric_comp
            .y
            .clamp(-BODY_ACCEL_CLAMP_LINEAR, BODY_ACCEL_CLAMP_LINEAR);
        let clamped_z = body_accel_fric_comp
            .z
            .clamp(-BODY_ACCEL_CLAMP_ANGULAR, BODY_ACCEL_CLAMP_ANGULAR);
        let accel_clamped = clamped_x != body_accel_fric_comp.x
            || clamped_y != body_accel_fric_comp.y
            || clamped_z != body_accel_fric_comp.z;
        self.body_accel_out_fric_comp = Vector3f::new(clamped_x, clamped_y, clamped_z);
        let body_xy = SVector::<f32, 2>::new(self.body_twist_out.x, self.body_twist_out.y);
        self.wheel_vel_out =
            if matches!(ENC_LAG_MODE, EncLagMode::FeedforwardOnly | EncLagMode::Full) {
                let compensated_xy = self.control_context.enc_lag.invert(&body_xy);
                let compensated_twist =
                    Vector3f::new(compensated_xy.x, compensated_xy.y, self.body_twist_out.z);
                self.control_context
                    .robot_model
                    .transform_twist2wheel(self.control_context.state_estimate.z)
                    * compensated_twist
            } else {
                self.control_context
                    .robot_model
                    .transform_twist2wheel(self.control_context.state_estimate.z)
                    * self.body_twist_out
            };

        self.wheel_torque_out = self
            .control_context
            .robot_model
            .transform_accel2wheel(self.control_context.state_estimate.z)
            * self.body_accel_out_fric_comp;

        if !matches!(ENC_LAG_MODE, EncLagMode::Disabled) {
            self.control_context.enc_lag.step(&body_xy);
        }

        let t_after_effort = Instant::now();

        let traj_pos: [f32; 3] = self
            .control_context
            .trajectory
            .as_ref()
            .map(|t| {
                let sample = t.sample();
                let p = sample.0.fixed_rows::<3>(0);
                [p[0], p[1], p[2]]
            })
            .unwrap_or_default();
        let traj_vel: [f32; 3] = self
            .control_context
            .trajectory
            .as_ref()
            .map(|t| {
                let sample = t.sample();
                let v = sample.0.fixed_rows::<3>(3);
                [v[0], v[1], v[2]]
            })
            .unwrap_or_default();
        let kf_pos_pred = state_prediction.fixed_rows::<3>(0);
        let kf_vel_pred = state_prediction.fixed_rows::<3>(3);
        let kf_pos_est = self.control_context.state_estimate.fixed_rows::<3>(0);
        let kf_vel_est = self.control_context.state_estimate.fixed_rows::<3>(3);
        self.debug_telemetry = BodyControlExtendedTelemetry {
            flags: BodyControlExtTelemetryFlags::default().with_vision_update(vision_update),
            _reserved: [0u8; 3],
            maneuver: self.debug_telemetry.maneuver,
            imu_gyro: [0.0, 0.0, imu_gyro_theta_meas],
            imu_accel: [imu_accel_x_meas, imu_accel_y_meas, 0.0],
            vision_pose: [vision_pose_meas.x, vision_pose_meas.y, vision_pose_meas.z],
            body_traj_pos: traj_pos,
            body_traj_vel: traj_vel,
            kf_body_pos_prediction: [kf_pos_pred[0], kf_pos_pred[1], kf_pos_pred[2]],
            kf_body_vel_prediction: [kf_vel_pred[0], kf_vel_pred[1], kf_vel_pred[2]],
            kf_body_pos_estimate: [kf_pos_est[0], kf_pos_est[1], kf_pos_est[2]],
            kf_body_vel_estimate: [kf_vel_est[0], kf_vel_est[1], kf_vel_est[2]],
            body_vel_u: [self.body_twist_out.x, self.body_twist_out.y, self.body_twist_out.z],
            body_accel_u: [self.body_accel_out.x, self.body_accel_out.y, self.body_accel_out.z],
            body_accel_u_fric_comp: [
                self.body_accel_out_fric_comp.x,
                self.body_accel_out_fric_comp.y,
                self.body_accel_out_fric_comp.z,
            ],
        };
        self.debug_telemetry.set_maneuver_telemetry(maneuver_telem);

        let t_after_telem = Instant::now();

        self.control_context
            .robot_model
            .kf_predict(self.body_accel_out);

        let t_after_kf_predict = Instant::now();
        if trace {
            defmt::trace!(
                "CONTROL UPDATE TRACE - KF update: {} us, effort compute: {} us, control outputs: {} us, KF predict: {} us",
                (t_after_kf_update - t_start).as_micros(),
                (t_after_effort - t_after_kf_update).as_micros(),
                (t_after_telem - t_after_effort).as_micros(),
                (t_after_kf_predict - t_after_telem).as_micros(),
            );
        }

        Ok((vel_clamped, accel_clamped))
    }

    pub fn get_wheel_velocities(&self) -> Vector4f {
        self.wheel_vel_out
    }

    pub fn get_wheel_torques(&self) -> Vector4f {
        self.wheel_torque_out
    }

    pub fn get_wheel_currents(&self) -> Vector4f {
        self.control_context
            .robot_model
            .torques_to_currents(self.wheel_torque_out)
    }

    pub fn get_control_telem(&self) -> BodyControlTelemetry {
        self.telemetry
    }

    pub fn get_control_debug_telem(&self) -> BodyControlExtendedTelemetry {
        self.debug_telemetry
    }
}

impl ParameterInterface for BodyController {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool {
        self.has_name(param_cmd.parameter_name)
    }

    fn has_name(&self, param_name: ParameterName) -> bool {
        ControlContext::expected_format(param_name).is_some()
    }

    fn apply_command(
        &mut self,
        param_cmd: &ParameterCommand,
    ) -> Result<ParameterCommand, ParameterCommand> {
        let mut reply = *param_cmd;

        if param_cmd.command_code != ParameterCommandCode::Read
            && param_cmd.command_code != ParameterCommandCode::Write
        {
            defmt::warn!("asked to apply a command without an actionable command code");
            return Err(reply);
        }

        let fmt = match ControlContext::expected_format(param_cmd.parameter_name) {
            Some(f) => f,
            None => {
                defmt::warn!(
                    "unexpected parameter name, cannot apply command"
                );
                reply.command_code = ParameterCommandCode::NackInvalidName;
                return Err(reply);
            }
        };

        if param_cmd.command_code == ParameterCommandCode::Read {
            defmt::info!("Reading parameter");
            self.control_context
                .read_param(param_cmd.parameter_name, &mut reply);
        } else {
            defmt::info!("Writing parameter");
            if core::mem::discriminant(&param_cmd.data) != core::mem::discriminant(&fmt) {
                reply.command_code = ParameterCommandCode::NackInvalidTypeForName;
                return Err(reply);
            }
            self.control_context.write_param(param_cmd);
            self.reset();
        }

        reply.command_code = ParameterCommandCode::Ack;
        Ok(reply)
    }
}
