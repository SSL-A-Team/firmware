use crate::motion::control_context::ControlContext;
use crate::motion::maneuvers::ManeuverManager;
use crate::motion::params::controller_params::{
    EncLagMode, BODY_ACCEL_CLAMP_ANGULAR, BODY_ACCEL_CLAMP_LINEAR, BODY_VEL_CLAMP_ANGULAR,
    BODY_VEL_CLAMP_LINEAR, ENC_LAG_MODE, STOP_STATE_LINEAR_SPEED_LIMIT,
};
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{
    BasicControl, BodyControlExtendedTelemetry, BodyControlTelemetry, ParameterCommand,
    ParameterCommandCode::*, ParameterName,
};
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

        let (setpoints, maneuver_telem) = self
            .maneuver_manager
            .tick(last_command, &mut self.control_context)?;

        self.body_twist_out = setpoints.body_twist;
        self.body_accel_out = setpoints.body_accel;

        // SSL stop state: clamp linear speed after control policy output so feedback
        // loops cannot overshoot to recover trajectory error.
        if last_command.game_state_in_stop() != 0 {
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
            body_traj_pos: self
                .control_context
                .trajectory
                .as_ref()
                .map(|t| {
                    let pos: Vector3f = t.sample().0.fixed_rows::<3>(0).into();
                    pos.into()
                })
                .unwrap_or_default(),
            body_traj_vel: self
                .control_context
                .trajectory
                .as_ref()
                .map(|t| {
                    let vel: Vector3f = t.sample().0.fixed_rows::<3>(3).into();
                    vel.into()
                })
                .unwrap_or_default(),
            kf_body_pos_prediction: state_prediction.fixed_rows::<3>(0).into(),
            kf_body_vel_prediction: state_prediction.fixed_rows::<3>(3).into(),
            kf_body_pos_estimate: self
                .control_context
                .state_estimate
                .fixed_rows::<3>(0)
                .into(),
            kf_body_vel_estimate: self
                .control_context
                .state_estimate
                .fixed_rows::<3>(3)
                .into(),
            body_vel_u: self.body_twist_out.into(),
            body_accel_u: self.body_accel_out.into(),
            body_accel_u_fric_comp: self.body_accel_out_fric_comp.into(),
            ..self.debug_telemetry
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

    fn has_name(&self, param_name: ParameterName::Type) -> bool {
        ControlContext::expected_format(param_name).is_some()
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

        let fmt = match ControlContext::expected_format(param_cmd.parameter_name) {
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
            self.control_context
                .read_param(param_cmd.parameter_name, &mut reply);
        } else {
            defmt::info!("Writing parameter {}", param_cmd.parameter_name);
            if param_cmd.data_format != fmt {
                reply.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                return Err(reply);
            }
            self.control_context.write_param(param_cmd);
            self.reset();
        }

        reply.command_code = PCC_ACK;
        Ok(reply)
    }
}
