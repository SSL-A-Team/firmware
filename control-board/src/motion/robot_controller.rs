use core::f32::consts::PI;

use crate::parameter_interface::ParameterInterface;
use crate::motion::pid::PidController;
use ateam_common_packets::bindings::{
    ParameterCommandCode::*, ParameterDataFormat, ParameterName
};
use ateam_controls::{Vector3f, Vector4f, Vector6f, Vector8f};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::robot_model::RobotModel;
use embassy_time::Instant;
use libm::{sqrtf, fabsf};
use nalgebra::{vector, Matrix3x5};

use ateam_common_packets::bindings::{ExtendedTelemetry, ParameterCommand};

/// Active control mode for pose control, used for bang-bang / PID hysteresis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PoseControlMode {
    /// Bang-bang trajectory controller (used for large errors)
    BangBang,
    /// PID controller (used once error is small)
    Pid,
}

/// Hysteresis thresholds for switching between bang-bang and PID control.
#[derive(Clone, Copy, Debug)]
pub struct PoseControlHysteresis {
    /// Linear position error below which we switch from bang-bang to PID
    pub pid_enter_error_pos_linear: f32,
    /// Angular position error below which we switch from bang-bang to PID
    pub pid_enter_error_pos_angular: f32,
    /// Linear position error above which we switch from PID back to bang-bang
    pub pid_exit_error_pos_linear: f32,
    /// Angular position error above which we switch from PID back to bang-bang
    pub pid_exit_error_pos_angular: f32,
    /// Linear velocity error below which we switch from bang-bang to PID
    pub pid_enter_error_vel_linear: f32,
    /// Angular velocity error below which we switch from bang-bang to PID
    pub pid_enter_error_vel_angular: f32,
    /// Linear velocity error above which we switch from PID back to bang-bang
    pub pid_exit_error_vel_linear: f32,
    /// Angular velocity error above which we switch from PID back to bang-bang
    pub pid_exit_error_vel_angular: f32,
}

impl Default for PoseControlHysteresis {
    fn default() -> Self {
        PoseControlHysteresis {
            pid_enter_error_pos_linear: 0.05,    // meters
            pid_enter_error_pos_angular: 0.2,  // radians
            pid_exit_error_pos_linear: 0.1,     // meters (larger than enter for hysteresis)
            pid_exit_error_pos_angular: 0.4,    // radians (larger than enter for hysteresis)
            pid_enter_error_vel_linear: 0.2,    // m/s
            pid_enter_error_vel_angular: 0.5,   // rad/s
            pid_exit_error_vel_linear: 0.4,     // m/s (larger than enter for hysteresis)
            pid_exit_error_vel_angular: 0.8,    // rad/s (larger than enter for hysteresis)
        }
    }
}


pub struct BodyController {
    pub robot_model: RobotModel,
    pub pose_pid_controller: PidController<3>,
    pub twist_pid_controller: PidController<3>,
    pub trajectory_params: TrajectoryParams,
    pub pose_control_mode: PoseControlMode,
    pub pose_control_hysteresis: PoseControlHysteresis,
    pub body_twist_cmd: Vector3f,
    pub body_accel_cmd: Vector3f,
    pub prev_body_twist_cmd: Vector3f,  // Used for twist PID control
    pub wheel_vel_cmd: Vector4f,
    pub wheel_torque_cmd: Vector4f,
    pub debug_telemetry: ExtendedTelemetry,
    pub dt: f32,
}

impl BodyController {
    pub fn new(dt: f32) -> BodyController {
        BodyController {
            robot_model: RobotModel::new_from_default_params(dt).expect("Failed to create RobotModel, check that default parameters are valid"),
            pose_pid_controller: PidController::<3>::from_gains_matrix(&Matrix3x5::zeros()),
            twist_pid_controller: PidController::<3>::from_gains_matrix(&Matrix3x5::zeros()),
            trajectory_params: TrajectoryParams::default(),
            pose_control_mode: PoseControlMode::BangBang,
            pose_control_hysteresis: PoseControlHysteresis::default(),
            body_twist_cmd: Vector3f::default(),
            body_accel_cmd: Vector3f::default(),
            prev_body_twist_cmd: Vector3f::default(),
            wheel_vel_cmd: Vector4f::default(),
            wheel_torque_cmd: Vector4f::default(),
            debug_telemetry: Default::default(),
            dt,
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

        // TODO: consider what to do here for singular matrix cases - maybe skip the update and just use the prediction as the estimate for this cycle, or add some regularization to the covariance to prevent singularities?
        let res = self.robot_model.kf_update(measurement, !vision_update, false, false);
        if res.is_err() {
            defmt::error!("Kalman filter update failed, skipping update: {}", res.err().unwrap() as i32);
        }

        let mut state_estimate = self.robot_model.x.clone();

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

        let kf_update_time = Instant::now() - start;
        start = Instant::now();

        let state_estimate = self.robot_model.x;

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
        (self.body_twist_cmd, self.body_accel_cmd) = self.pose_pid_control(state_estimate, target_pose);
        // (self.body_twist_cmd, self.body_accel_cmd) = self.pose_bangbang_control(state_estimate, target_pose);
        // (self.body_twist_cmd, self.body_accel_cmd) = self.pose_dualmode_control(state_estimate, target_pose);
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_cmd;
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * self.body_accel_cmd;
    }

    pub fn compute_effort_twist_control(&mut self, state_estimate: Vector6f, target_twist: Vector3f) {
        // (self.body_twist_cmd, self.body_accel_cmd) = self.twist_bangbang_control(state_estimate, target_twist);
        (self.body_twist_cmd, self.body_accel_cmd) = self.twist_pid_control(state_estimate, target_twist);
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_cmd;
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * self.body_accel_cmd;
    }

    pub fn compute_effort_accel_control(&mut self, state_estimate: Vector6f, target_accel: Vector3f) {
        let next_state = self.robot_model.a * state_estimate + self.robot_model.b * target_accel;
        self.body_twist_cmd = next_state.fixed_rows::<3>(3).into();
        self.body_accel_cmd = target_accel;
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * self.body_twist_cmd;
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * self.body_accel_cmd;
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

    fn pose_pid_control(&mut self, state_estimate: Vector6f, target_pose: Vector3f) -> (Vector3f, Vector3f) {
        let global_accel_cmd = self.pose_pid_controller.calculate(
            &target_pose, 
            &state_estimate.fixed_rows::<3>(0).into(),
            self.dt
        );
        let global_twist_cmd = state_estimate.fixed_rows::<3>(3) + self.body_accel_cmd * self.dt;
        (global_twist_cmd, global_accel_cmd)
    }

    fn pose_bangbang_control(&mut self, state_estimate: Vector6f, target_pose: Vector3f) -> (Vector3f, Vector3f) {
        // Calculate the optimal trajectory to the setpoint
        let traj = BangBangTraj3D::from_target_pose(
            state_estimate, target_pose, self.trajectory_params,
        ).expect("Failed to generate bang-bang trajectory, check that trajectory parameters are valid");
        // Calculate the acceleration needed to achieve the trajectory right now
        let global_accel_cmd = traj.accel_at(0.0).expect("Bang-bang trajectory should always have a valid accel at t=0.0");
        // Calculate the twist that should be achieved at the next time step after applying this acceleration
        let next_body_state = traj.state_at(state_estimate, 0.0, self.dt).expect("Bang-bang trajectory should always have a valid state at t=0.0 + dt");
        let global_twist_cmd = Vector3f::new(next_body_state[3], next_body_state[4], next_body_state[5]);
        (global_twist_cmd, global_accel_cmd)
    }

    fn pose_dualmode_control(&mut self, state_estimate: Vector6f, target_pose: Vector3f) -> (Vector3f, Vector3f) {
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let pose_error: Vector3f = target_pose - pose_estimate;
        let linear_error = sqrtf(pose_error.x * pose_error.x + pose_error.y * pose_error.y);
        let angular_error = pose_error.z.abs();
        // Target pose implies zero velocity, so velocity error is the current velocity magnitude
        let linear_vel_error = sqrtf(twist_estimate.x * twist_estimate.x + twist_estimate.y * twist_estimate.y);
        let angular_vel_error = twist_estimate.z.abs();

        let hyst = &self.pose_control_hysteresis;

        // Dual-mode switching with hysteresis:
        //  - Switch to PID when both position and velocity errors drop below the enter thresholds
        //  - Switch back to bang-bang when any position or velocity error rises above the exit thresholds
        match self.pose_control_mode {
            PoseControlMode::BangBang => {
                if linear_error < hyst.pid_enter_error_pos_linear
                    && angular_error < hyst.pid_enter_error_pos_angular
                    && linear_vel_error < hyst.pid_enter_error_vel_linear
                    && angular_vel_error < hyst.pid_enter_error_vel_angular
                {
                    self.pose_control_mode = PoseControlMode::Pid;
                    self.pose_pid_controller.reset();
                }
            }
            PoseControlMode::Pid => {
                if linear_error > hyst.pid_exit_error_pos_linear
                    || angular_error > hyst.pid_exit_error_pos_angular
                    || linear_vel_error > hyst.pid_exit_error_vel_linear
                    || angular_vel_error > hyst.pid_exit_error_vel_angular
                {
                    self.pose_control_mode = PoseControlMode::BangBang;
                }
            }
        }

        match self.pose_control_mode {
            PoseControlMode::BangBang => {
                self.pose_bangbang_control(state_estimate, target_pose)
            }
            PoseControlMode::Pid => {
                self.pose_pid_control(state_estimate, target_pose)
            }
        }
    }

    fn twist_bangbang_control(&mut self, state_estimate: Vector6f, target_twist: Vector3f) -> (Vector3f, Vector3f) {
        let traj = BangBangTraj3D::from_target_twist(
            state_estimate.fixed_rows::<3>(3).into(),
            target_twist,
            self.trajectory_params,
        ).expect("Failed to generate bang-bang trajectory, check that trajectory parameters are valid");
        let next_state = traj.state_at(state_estimate, 0.0, self.dt).expect("Bang-bang trajectory should always have a valid state at t=0.0 + dt");
        let global_twist_cmd: Vector3f = next_state.fixed_rows::<3>(3).into();
        let global_accel_cmd = traj.accel_at(0.0).expect("Bang-bang trajectory should always have a valid accel at t=0.0");
        (global_twist_cmd, global_accel_cmd)
    }

    fn twist_pid_control(&mut self, state_estimate: Vector6f, target_twist: Vector3f) -> (Vector3f, Vector3f) {
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let body_twist_pid = self.twist_pid_controller.calculate(&target_twist, &twist_estimate, self.dt);
        let mut global_twist_cmd = target_twist + body_twist_pid;

        // Determine commanded body acceleration based on previous control output, and clamp and maintain the direction of acceleration.
        // NOTE: Using previous control output instead of estimate so that collision disturbances would not impact.
        let mut global_accel_cmd = (global_twist_cmd - self.prev_body_twist_cmd) / self.dt;

        let max_accel_linear = self.trajectory_params.max_accel_linear;
        let max_accel_angular = self.trajectory_params.max_accel_angular;
        let max_vel_linear = self.trajectory_params.max_vel_linear;
        let max_vel_angular = self.trajectory_params.max_vel_angular;

        // Clamp acceleration: linear magnitude and angular independently
        let accel_linear_mag = sqrtf(global_accel_cmd.x * global_accel_cmd.x + global_accel_cmd.y * global_accel_cmd.y);
        if accel_linear_mag > max_accel_linear {
            let scale = max_accel_linear / accel_linear_mag;
            global_accel_cmd.x *= scale;
            global_accel_cmd.y *= scale;
        }
        global_accel_cmd.z = global_accel_cmd.z.clamp(-max_accel_angular, max_accel_angular);

        // Recompute twist from clamped acceleration
        global_twist_cmd = self.prev_body_twist_cmd + (global_accel_cmd * self.dt);

        // Clamp twist: linear magnitude and angular independently
        let twist_linear_mag = sqrtf(global_twist_cmd.x * global_twist_cmd.x + global_twist_cmd.y * global_twist_cmd.y);
        if twist_linear_mag > max_vel_linear {
            let scale = max_vel_linear / twist_linear_mag;
            global_twist_cmd.x *= scale;
            global_twist_cmd.y *= scale;
        }
        global_twist_cmd.z = global_twist_cmd.z.clamp(-max_vel_angular, max_vel_angular);

        // Recompute accel to stay consistent with the clamped twist
        global_accel_cmd = (global_twist_cmd - self.prev_body_twist_cmd) / self.dt;

        self.prev_body_twist_cmd.copy_from_slice(global_twist_cmd.as_slice());

        (global_twist_cmd, global_accel_cmd)
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
            ParameterName::PIDII_XD => true,
            ParameterName::PIDII_YD => true,
            ParameterName::PIDII_THETAD => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR => true,
            ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR => true,
            ParameterName::TRAJ_MAX_VEL_LINEAR => true,
            ParameterName::TRAJ_MAX_VEL_ANGULAR => true,
            ParameterName::TRAJ_MAX_ACCEL_LINEAR => true,
            ParameterName::TRAJ_MAX_ACCEL_ANGULAR => true,
            ParameterName::HYST_PID_ENTER_ERROR_POS_LINEAR => true,
            ParameterName::HYST_PID_ENTER_ERROR_POS_ANGULAR => true,
            ParameterName::HYST_PID_EXIT_ERROR_POS_LINEAR => true,
            ParameterName::HYST_PID_EXIT_ERROR_POS_ANGULAR => true,
            ParameterName::HYST_PID_ENTER_ERROR_VEL_LINEAR => true,
            ParameterName::HYST_PID_ENTER_ERROR_VEL_ANGULAR => true,
            ParameterName::HYST_PID_EXIT_ERROR_VEL_LINEAR => true,
            ParameterName::HYST_PID_EXIT_ERROR_VEL_ANGULAR => true,
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
                | ParameterName::PHYS_MOTOR_EFFICIENCY_FACTOR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR
                | ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR
                | ParameterName::TRAJ_MAX_VEL_LINEAR
                | ParameterName::TRAJ_MAX_VEL_ANGULAR
                | ParameterName::TRAJ_MAX_ACCEL_LINEAR
                | ParameterName::TRAJ_MAX_ACCEL_ANGULAR
                | ParameterName::HYST_PID_ENTER_ERROR_POS_LINEAR
                | ParameterName::HYST_PID_ENTER_ERROR_POS_ANGULAR
                | ParameterName::HYST_PID_EXIT_ERROR_POS_LINEAR
                | ParameterName::HYST_PID_EXIT_ERROR_POS_ANGULAR
                | ParameterName::HYST_PID_ENTER_ERROR_VEL_LINEAR
                | ParameterName::HYST_PID_ENTER_ERROR_VEL_ANGULAR
                | ParameterName::HYST_PID_EXIT_ERROR_VEL_LINEAR
                | ParameterName::HYST_PID_EXIT_ERROR_VEL_ANGULAR => {
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
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR => self.trajectory_params.allowable_error_pos_linear,
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR => self.trajectory_params.allowable_error_pos_angular,
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR => self.trajectory_params.allowable_error_vel_linear,
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR => self.trajectory_params.allowable_error_vel_angular,
                        ParameterName::TRAJ_MAX_VEL_LINEAR => self.trajectory_params.max_vel_linear,
                        ParameterName::TRAJ_MAX_VEL_ANGULAR => self.trajectory_params.max_vel_angular,
                        ParameterName::TRAJ_MAX_ACCEL_LINEAR => self.trajectory_params.max_accel_linear,
                        ParameterName::TRAJ_MAX_ACCEL_ANGULAR => self.trajectory_params.max_accel_angular,
                        ParameterName::HYST_PID_ENTER_ERROR_POS_LINEAR => self.pose_control_hysteresis.pid_enter_error_pos_linear,
                        ParameterName::HYST_PID_ENTER_ERROR_POS_ANGULAR => self.pose_control_hysteresis.pid_enter_error_pos_angular,
                        ParameterName::HYST_PID_EXIT_ERROR_POS_LINEAR => self.pose_control_hysteresis.pid_exit_error_pos_linear,
                        ParameterName::HYST_PID_EXIT_ERROR_POS_ANGULAR => self.pose_control_hysteresis.pid_exit_error_pos_angular,
                        ParameterName::HYST_PID_ENTER_ERROR_VEL_LINEAR => self.pose_control_hysteresis.pid_enter_error_vel_linear,
                        ParameterName::HYST_PID_ENTER_ERROR_VEL_ANGULAR => self.pose_control_hysteresis.pid_enter_error_vel_angular,
                        ParameterName::HYST_PID_EXIT_ERROR_VEL_LINEAR => self.pose_control_hysteresis.pid_exit_error_vel_linear,
                        ParameterName::HYST_PID_EXIT_ERROR_VEL_ANGULAR => self.pose_control_hysteresis.pid_exit_error_vel_angular,
                        _ => unreachable!(),
                    };
                },
                ParameterName::PIDII_X
                | ParameterName::PIDII_Y
                | ParameterName::PIDII_THETA => {
                    reply_cmd.data_format = ParameterDataFormat::PID_LIMITED_INTEGRAL_F32;
                    let gain = self.pose_pid_controller.get_gain();
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_X => 0,
                        ParameterName::PIDII_Y => 1,
                        ParameterName::PIDII_THETA => 2,
                        _ => unreachable!(),
                    };
                    reply_cmd.data.pidii_f32 = [gain[(row, 0)], gain[(row, 1)], gain[(row, 2)], gain[(row, 3)], gain[(row, 4)]];
                },
                ParameterName::PIDII_XD
                | ParameterName::PIDII_YD
                | ParameterName::PIDII_THETAD => {
                    reply_cmd.data_format = ParameterDataFormat::PID_LIMITED_INTEGRAL_F32;
                    let gain = self.twist_pid_controller.get_gain();
                    let row = match param_cmd.parameter_name {
                        ParameterName::PIDII_XD => 0,
                        ParameterName::PIDII_YD => 1,
                        ParameterName::PIDII_THETAD => 2,
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
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_LINEAR => self.trajectory_params.allowable_error_pos_linear = write_value,
                        ParameterName::TRAJ_ALLOWABLE_ERROR_POS_ANGULAR => self.trajectory_params.allowable_error_pos_angular = write_value,
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_LINEAR => self.trajectory_params.allowable_error_vel_linear = write_value,
                        ParameterName::TRAJ_ALLOWABLE_ERROR_VEL_ANGULAR => self.trajectory_params.allowable_error_vel_angular = write_value,
                        ParameterName::TRAJ_MAX_VEL_LINEAR => self.trajectory_params.max_vel_linear = write_value,
                        ParameterName::TRAJ_MAX_VEL_ANGULAR => self.trajectory_params.max_vel_angular = write_value,
                        ParameterName::TRAJ_MAX_ACCEL_LINEAR => self.trajectory_params.max_accel_linear = write_value,
                        ParameterName::TRAJ_MAX_ACCEL_ANGULAR => self.trajectory_params.max_accel_angular = write_value,
                        _ => unreachable!(),
                    }
                },
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
                        ParameterName::HYST_PID_ENTER_ERROR_POS_LINEAR => self.pose_control_hysteresis.pid_enter_error_pos_linear = write_value,
                        ParameterName::HYST_PID_ENTER_ERROR_POS_ANGULAR => self.pose_control_hysteresis.pid_enter_error_pos_angular = write_value,
                        ParameterName::HYST_PID_EXIT_ERROR_POS_LINEAR => self.pose_control_hysteresis.pid_exit_error_pos_linear = write_value,
                        ParameterName::HYST_PID_EXIT_ERROR_POS_ANGULAR => self.pose_control_hysteresis.pid_exit_error_pos_angular = write_value,
                        ParameterName::HYST_PID_ENTER_ERROR_VEL_LINEAR => self.pose_control_hysteresis.pid_enter_error_vel_linear = write_value,
                        ParameterName::HYST_PID_ENTER_ERROR_VEL_ANGULAR => self.pose_control_hysteresis.pid_enter_error_vel_angular = write_value,
                        ParameterName::HYST_PID_EXIT_ERROR_VEL_LINEAR => self.pose_control_hysteresis.pid_exit_error_vel_linear = write_value,
                        ParameterName::HYST_PID_EXIT_ERROR_VEL_ANGULAR => self.pose_control_hysteresis.pid_exit_error_vel_angular = write_value,
                        _ => unreachable!(),
                    }
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
                    let mut gain = self.pose_pid_controller.get_gain();
                    for col in 0..5 {
                        gain[(row, col)] = write_values[col];
                    }
                    self.pose_pid_controller.set_gain(gain);
                    self.pose_pid_controller.reset();
                    reply_cmd.data.pidii_f32 = write_values;
                },
                ParameterName::PIDII_XD
                | ParameterName::PIDII_YD
                | ParameterName::PIDII_THETAD => {
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
                    self.twist_pid_controller.reset();
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
