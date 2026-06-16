use crate::motion::control_context::ControlContext;
use crate::motion::maneuvers::ManeuverManager;
use crate::motion::params::controller_params::{EncLagMode, ENC_LAG_MODE};
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{
    BasicControl, BodyControlExtendedTelemetry, BodyControlTelemetry, ParameterCommand,
    ParameterCommandCode::*, ParameterName,
};
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
    ) -> Result<(), ControlsError> {
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

        let friction_force_global = self
            .control_context
            .compute_friction(self.body_twist_out, self.body_accel_out);
        self.body_accel_out_fric_comp =
            self.body_accel_out - self.control_context.robot_model.i_inv * friction_force_global;

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
                .trajectory_state
                .fixed_rows::<3>(0)
                .into(),
            body_traj_vel: self
                .control_context
                .trajectory_state
                .fixed_rows::<3>(3)
                .into(),
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

        Ok(())
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
