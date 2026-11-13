use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackingDivergenceState};
use crate::motion::maneuvers::{requires_vision, ManeuverManager};
use crate::motion::params::controller_params::{
    EncLagMode, BODY_ACCEL_CLAMP_ANGULAR, BODY_ACCEL_CLAMP_LINEAR, BODY_VEL_CLAMP_ANGULAR,
    BODY_VEL_CLAMP_LINEAR, ENC_LAG_MODE, STOP_STATE_LINEAR_SPEED_LIMIT,
    FRICTION_VEL_LPF_HZ, WHEEL_CURRENT_SLOPE_LIMIT_A_PER_S,
};
use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::BodyControlMode::{BCM_ESTOP_BRAKE, BCM_OFF};
use ateam_common_packets::bindings::{
    BasicControl, BodyControlExtendedTelemetry, BodyControlTelemetry, ParameterCommand,
    ParameterCommandCode::*, ParameterName,
};
use ateam_common_packets::radio::ManeuverExtendedTelemetry;
use ateam_controls::trajectory::Trajectory;
use ateam_controls::{z_rotation_mat, ControlsError, Vector3f, Vector4f};
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
    /// Slope-limited wheel current output (A); the slope limiter's state.
    pub wheel_current_out: Vector4f,
    /// Low-passed local commanded velocity used only for friction compensation.
    friction_vel_local_filt: Vector3f,
    friction_vel_alpha: f32,
    pub telemetry: BodyControlTelemetry,
    pub debug_telemetry: BodyControlExtendedTelemetry,
    /// Commanded maneuver requires vision and vision is inactive: maneuvers are
    /// held and the control task active-brakes.
    pub vision_hold: bool,
}

impl BodyController {
    pub fn new(dt: f32) -> BodyController {
        BodyController {
            control_context: ControlContext::new(dt),
            maneuver_manager: ManeuverManager::new(),
            vision_hold: false,
            body_twist_out: Vector3f::default(),
            body_accel_out: Vector3f::default(),
            body_accel_out_fric_comp: Vector3f::default(),
            wheel_vel_out: Vector4f::default(),
            wheel_torque_out: Vector4f::default(),
            wheel_current_out: Vector4f::default(),
            friction_vel_local_filt: Vector3f::default(),
            friction_vel_alpha: {
                let tau = 1.0 / (2.0 * core::f32::consts::PI * FRICTION_VEL_LPF_HZ);
                dt / (tau + dt)
            },
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
        self.wheel_current_out = Vector4f::default();
        self.friction_vel_local_filt = Vector3f::default();
        self.telemetry = Default::default();
        self.debug_telemetry = Default::default();
    }

    pub fn vision_active(&self) -> bool {
        self.control_context.vision_active()
    }

    /// True while recovering from a trajectory divergence (e.g. a collision); the
    /// control task should command the active brake. Reset happens automatically
    /// once the wheels stop.
    pub fn tracking_divergence_recovery_active(&self) -> bool {
        self.control_context.tracking_divergence_state == TrackingDivergenceState::Recovering
    }

    /// True when the control task should command the active brake instead of the
    /// maneuver outputs: divergence recovery, or a vision-required maneuver is
    /// commanded while vision is inactive.
    pub fn active_brake_requested(&self) -> bool {
        self.tracking_divergence_recovery_active() || self.vision_hold
    }

    pub fn control_update(
        &mut self,
        last_command: BasicControl,
        vision_pose_meas: Vector3f,
        vision_t_capture_host_us: u64,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
        imu_accel_x_meas: f32,
        imu_accel_y_meas: f32,
        trace: bool,
    ) -> Result<(bool, bool), ControlsError> {
        let t_start = Instant::now();

        let state_prediction = self.control_context.update_state_estimate(
            vision_pose_meas,
            vision_t_capture_host_us,
            vision_update,
            wheel_vel_meas,
            imu_gyro_theta_meas,
            imu_accel_x_meas,
            imu_accel_y_meas,
        )?;

        let t_after_state_est = Instant::now();

        // While the robot is halted (SSL HALT game state), recovering from a
        // trajectory divergence (e.g. a collision), or commanded into a maneuver
        // that requires vision while vision is inactive, the control task ignores the
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
        self.vision_hold = requires_vision(&last_command.get_maneuver_command())
            && !self.control_context.vision_active();
        let hold_maneuvers = self.vision_hold
            || last_command.game_state_in_halt() != 0
            || self.control_context.tracking_divergence_state
                == TrackingDivergenceState::Recovering
            || last_command.body_control_mode == BCM_OFF
            || last_command.body_control_mode == BCM_ESTOP_BRAKE;

        let (setpoints, maneuver_telem) = if hold_maneuvers {
            self.maneuver_manager.reset();
            self.control_context.reset_trajectory();
            (ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off)
        } else {
            self.maneuver_manager
                .tick(last_command, &mut self.control_context)?
        };

        // Trajectory-divergence recovery: a large unexpected tracking error (e.g.
        // a collision) trips into a braking recovery; the controller resets only
        // after braking finishes, then tracking replans from the fresh estimate.
        self.control_context
            .update_tracking_divergence_recovery(wheel_vel_meas);

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

        // Feed-forward output stage after TIGERs Mannheim `CtrlPantheraUpdate`
        // (Firmware src/robot/ctrl_panthera.c): in the local frame (heading
        // advanced by half a period), add the friction compensation for the
        // commanded velocity to the commanded acceleration, divide by the
        // output efficiency, convert to wheel torque, then slope-limit the
        // wheel current.
        let output_heading = self.control_context.output_heading();
        let to_local = z_rotation_mat(-output_heading);
        let robot_model = &self.control_context.robot_model;
        let twist_local: Vector3f = to_local * self.body_twist_out;
        let accel_local: Vector3f = to_local * self.body_accel_out;
        // The commanded velocity carries the feedback terms (gyro / pose noise); the steep
        // Coulomb ramp would pass that straight into the current, so low-pass it here.
        self.friction_vel_local_filt +=
            (twist_local - self.friction_vel_local_filt) * self.friction_vel_alpha;
        // Friction and acceleration parts of the feed-forward, each already divided
        // by the output efficiency (TIGERs: (acc + F_fric/m) / efficiency).
        let fric_accel_local =
            robot_model.feedforward_local_accel(self.friction_vel_local_filt, Vector3f::zeros());
        let accel_ff_local = robot_model.feedforward_local_accel(Vector3f::zeros(), accel_local);

        // Clamp the body-level acceleration before converting to wheel torques.
        let clamped_accel_local = Vector3f::new(
            accel_ff_local
                .x
                .clamp(-BODY_ACCEL_CLAMP_LINEAR, BODY_ACCEL_CLAMP_LINEAR),
            accel_ff_local
                .y
                .clamp(-BODY_ACCEL_CLAMP_LINEAR, BODY_ACCEL_CLAMP_LINEAR),
            accel_ff_local
                .z
                .clamp(-BODY_ACCEL_CLAMP_ANGULAR, BODY_ACCEL_CLAMP_ANGULAR),
        );
        let accel_clamped = clamped_accel_local != accel_ff_local;

        let body_xy = SVector::<f32, 2>::new(self.body_twist_out.x, self.body_twist_out.y);
        let vel_cmd_global =
            if matches!(ENC_LAG_MODE, EncLagMode::FeedforwardOnly | EncLagMode::Full) {
                let compensated_xy = self.control_context.enc_lag.invert(&body_xy);
                Vector3f::new(compensated_xy.x, compensated_xy.y, self.body_twist_out.z)
            } else {
                self.body_twist_out
            };
        self.wheel_vel_out = robot_model.transform_vel2wheel_local() * (to_local * vel_cmd_global);

        // Fit the feed-forward into the per-wheel current budgets (thermally derated,
        // see wheel_thermal.rs) at the body level:
        // friction compensation first (it is what lets the robot move at all), then
        // the largest uniform scale s in [0, 1] of the acceleration vector for which
        // every wheel stays within the limit. Direction is preserved, and unlike
        // scaling the sum, saturation never eats the friction compensation.
        let accel_to_current = |a: &Vector3f| {
            robot_model.torques_to_currents(robot_model.transform_accel2wheel_local() * a)
        };
        let i_fric = accel_to_current(&fric_accel_local);
        let i_accel = accel_to_current(&clamped_accel_local);
        let limits = self.control_context.wheel_ff_current_limits;
        let mut fric_scale: f32 = 1.0;
        for i in 0..4 {
            if i_fric[i].abs() > limits[i] {
                fric_scale = fric_scale.min(limits[i] / i_fric[i].abs());
            }
        }
        let (fric_scale, accel_scale) = if fric_scale < 1.0 {
            (fric_scale, 0.0)
        } else {
            let mut s: f32 = 1.0;
            for i in 0..4 {
                let headroom = if i_accel[i] > 0.0 {
                    limits[i] - i_fric[i]
                } else {
                    limits[i] + i_fric[i]
                };
                if i_accel[i].abs() * s > headroom {
                    s = headroom / i_accel[i].abs();
                }
            }
            (1.0, s.clamp(0.0, 1.0))
        };
        let current_limited = fric_scale < 1.0 || accel_scale < 1.0;
        let target_currents = i_fric * fric_scale + i_accel * accel_scale;
        self.body_accel_out_fric_comp = z_rotation_mat(output_heading)
            * (fric_accel_local * fric_scale + clamped_accel_local * accel_scale);
        let accel_clamped = accel_clamped || current_limited;
        let max_step = WHEEL_CURRENT_SLOPE_LIMIT_A_PER_S * self.control_context.dt;
        for i in 0..4 {
            let step = (target_currents[i] - self.wheel_current_out[i]).clamp(-max_step, max_step);
            self.wheel_current_out[i] += step;
        }
        self.wheel_torque_out =
            self.wheel_current_out * robot_model.get_params().motor_torque_constant_nmpa;
        // Under load the wheel surface outruns the ground (omni rollers on carpet),
        // roughly in proportion to the wheel force. Without this the wheel velocity
        // loop holds the wheel at the kinematic setpoint and the body falls behind.
        self.wheel_vel_out += self.wheel_current_out * self.control_context.wheel_slip_comp;

        if !matches!(ENC_LAG_MODE, EncLagMode::Disabled) {
            self.control_context.enc_lag.step(&body_xy);
        }

        let t_after_effort = Instant::now();

        // Report vision in telemetry when the buffered EKF applies the
        // measurement. A measurement present at the horizon but too old to use
        // is reported via `vision_old` instead of `vision_update`.
        let applied_vision = self.control_context.applied_vision_measurement();
        let vision_update = matches!(applied_vision, Some((_, _, true)));
        let vision_old = matches!(applied_vision, Some((_, _, false)));
        let vision_insert_age_us = applied_vision.map(|(_, age_us, _)| age_us).unwrap_or(0);
        let vision_pose = applied_vision
            .map(|(meas, _, _)| meas)
            .unwrap_or(vision_pose_meas);

        self.debug_telemetry = BodyControlExtendedTelemetry {
            _bitfield_align_1: Default::default(),
            _bitfield_1: BodyControlExtendedTelemetry::new_bitfield_1(
                vision_update as u8,
                vision_old as u8,
                Default::default(),
            ),
            _reserved2: Default::default(),
            vision_insert_age_us,
            imu_gyro: [0.0, 0.0, imu_gyro_theta_meas],
            imu_accel: [imu_accel_x_meas, imu_accel_y_meas, 0.0],
            vision_pose: vision_pose.into(),
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
            kf_body_vel_estimate: self.control_context.state_estimate_vel_local.into(),
            body_vel_u: self.body_twist_out.into(),
            body_accel_u: self.body_accel_out.into(),
            body_accel_u_fric_comp: self.body_accel_out_fric_comp.into(),
            ..self.debug_telemetry
        };
        self.debug_telemetry.set_maneuver_telemetry(maneuver_telem);

        let t_after_telem = Instant::now();
        if trace {
            defmt::trace!(
                "CONTROL UPDATE TRACE - state estimate: {} us, effort compute: {} us, control outputs: {} us",
                (t_after_state_est - t_start).as_micros(),
                (t_after_effort - t_after_state_est).as_micros(),
                (t_after_telem - t_after_effort).as_micros(),
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
        self.wheel_current_out
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
