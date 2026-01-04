use crate::parameter_interface::ParameterInterface;
use ateam_common_packets::bindings::{
    ParameterCommandCode::*,
    ParameterDataFormat::{PID_LIMITED_INTEGRAL_F32, VEC3_F32, VEC4_F32},
    ParameterName,
    ParameterName::{
        ANGULAR_VEL_PID_Z, RC_BODY_ACC_LIMIT, RC_BODY_VEL_LIMIT, RC_WHEEL_ACC_LIMIT,
        VEL_CGFK_INITIAL_COVARIANCE, VEL_CGKF_ENCODER_NOISE, VEL_CGKF_GYRO_NOISE,
        VEL_CGKF_K_MATRIX, VEL_CGKF_PROCESS_NOISE, VEL_PID_X, VEL_PID_Y,
    },
};
use ateam_controls::{Vector3f, Vector4f, Vector6f, Vector8f};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::bangbang_trajectory::BangBangTraj3D;
use embassy_stm32::pac::adc::vals::Exten;
use embassy_time::{Duration, Instant};
use nalgebra::{vector, SVector, Vector3, Vector4, Vector5};

use super::constant_gain_kalman_filter::CgKalmanFilter;
use super::pid::PidController;

use super::params::body_vel_filter_params::{
    CONTROL_INPUT, INIT_ESTIMATE_COV, KALMAN_GAIN, KF_NUM_CONTROL_INPUTS, KF_NUM_OBSERVATIONS,
    KF_NUM_STATES, OBSERVATION_MODEL, PROCESS_COV, STATE_TRANSITION,
};

use super::params::body_vel_pid_params::{
    BODY_ACC_LIM, BODY_DEACC_LIM, BODY_VEL_LIM, PID_GAIN, WHEEL_ACC_LIM,
};

use ateam_common_packets::bindings::{ExtendedTelemetry, ParameterCommand};

// TODO find some general numeric type trait(s) for D
// Clamp the vector, but keep the direction consistent.
pub fn clamp_vector_keep_dir<const D: usize>(
    vec: &SVector<f32, D>,
    limits_abs: &SVector<f32, D>,
) -> SVector<f32, D> {
    // Applies the clamping in a sign-less way by getting the scale of the vector compared to absolute value clamp region.
    let scales: SVector<f32, D> = vec.abs().component_div(limits_abs);
    // To maintain direction, do a scalar divison of the max of the scales. Limit to make sure to only scale down.
    vec / f32::max(1.0, scales.max())
}


// TODO: Use CgKalman filter for body velocity estimate
pub struct BodyPoseController {
    robot_model: RobotModel,
    // body_vel_filter: CgKalmanFilter<'a, 3, 4, 5>,
    // body_vel_controller: PidController<3>,
    // body_velocity_limit: Vector3<f32>,
    // body_acceleration_limit: Vector3<f32>,
    // body_deceleration_limit: Vector3<f32>,
    // wheel_acceleration_limits: Vector4<f32>,
    // prev_output: Vector3<f32>,
    wheel_vel_cmd: Vector4f,
    wheel_torque_cmd: Vector4f,
    debug_telemetry: ExtendedTelemetry,
}

impl BodyPoseController {
    pub fn new(
    ) -> BodyPoseController {
        BodyPoseController {
            robot_model: RobotModel::new_from_constants(0.001),
            wheel_vel_cmd: Vector4f::default(),
            wheel_torque_cmd: Vector4f::default(),
            debug_telemetry: Default::default(),
        }
    }

    pub fn control_update(
        &mut self,
        pose_cmd: Vector3f,
        loop_period: Duration,
        vision_pose_meas: Vector3f,
        vision_pose_meas_instant: Instant,
        wheel_vel_meas: Vector4f,
        wheel_torque_meas: Vector4f,
        gyro_theta_meas: f32,
    ) {

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
        self.robot_model.kf_update(measurement, false, false, false);
        // let measurement: Vector5<f32> = Vector5::new(
        //     wheel_velocities_meas[0],
        //     wheel_velocities_meas[1],
        //     wheel_velocities_meas[2],
        //     wheel_velocities_meas[3],
        //     gyro_meas,
        // );

        // TODO: Use CgKalman filter for body velocity estimate
        // // Update measurements process observation input into CGKF.
        // self.body_vel_filter.update(&measurement);

        // // Read the current body velocity state estimate from the CGKF.
        // let mut body_vel_estimate = self.body_vel_filter.get_state();

        // // Deadzone the velocity estimate
        // if libm::fabsf(body_vel_estimate[0]) < 0.05 {
        //     body_vel_estimate[0] = 0.0;
        // }

        // if libm::fabsf(body_vel_estimate[1]) < 0.05 {
        //     body_vel_estimate[1] = 0.0;
        // }

        // if libm::fabsf(body_vel_estimate[2]) < 0.05 {
        //     body_vel_estimate[2] = 0.0;
        // }

        // self.debug_telemetry
        //     .cgkf_body_velocity_state_estimate
        //     .copy_from_slice(body_vel_estimate.as_slice());

        // // Use raw wheel readings and vision measurements for state estimate until kalman filter is implemented
        // let global_twist = self.robot_model.transform_wheel2twist(vision_pose_meas[2]) * wheel_vel_meas;

        // let state_estimate = Vector6f::new(
        //     vision_pose_meas[0],
        //     vision_pose_meas[1],
        //     vision_pose_meas[2],
        //     global_twist[0],
        //     global_twist[1],
        //     global_twist[2],
        // );
        let state_estimate = self.robot_model.x;
        // defmt::info!("State estimate pose: {}, {}, {}", state_estimate.x, state_estimate.y, state_estimate.z);
        // defmt::info!("State estimate twist: {}, {}, {}", state_estimate.w, state_estimate.a, state_estimate.b);

        let start = Instant::now();
        // Calculate global values
        // Calculate the optimal trajectory to the setpoint
        let traj = ateam_controls::bangbang_trajectory::compute_optimal_bangbang_traj_3d(
            state_estimate, pose_cmd,
        );
        // defmt::info!("Time elapsed in BodyPoseController control update: {} us", (Instant::now() - start).as_micros());
        // Calculate the acceleration needed to achieve the trajectory right now
        let global_accel_cmd = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_accel_at_t(traj, 0.0);
        // Calculate the twist that should be achieved at the next time step after applying this acceleration
        let next_body_state = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_state_at_t(traj, state_estimate, 0.0, 50.0 * loop_period.as_micros() as f32 * 1e-6);
        let global_twist_cmd = Vector3f::new(next_body_state[3], next_body_state[4], next_body_state[5]);

        // These torques are discretized by the loop rate, but in an ideal world, it would be a continuous command update to reach the next state wheel velocities as theta changes. However, the loop rate should be fast enough that the error due to a change in theta during each control period should be negligible, and the individual wheel velocities are achieved by the next control update
        self.wheel_torque_cmd = self.robot_model.transform_accel2wheel(state_estimate.z) * global_accel_cmd;
        self.wheel_vel_cmd = self.robot_model.transform_twist2wheel(state_estimate.z) * global_twist_cmd;
        // defmt::info!("Wheel Velocity Cmds: {}, {}, {}, {}", self.wheel_vel_cmd.x, self.wheel_vel_cmd.y, self.wheel_vel_cmd.z, self.wheel_vel_cmd.w);

        // Safety checks on commands
        if self.wheel_vel_cmd.x > 50.0 || self.wheel_vel_cmd.y > 50.0 || self.wheel_vel_cmd.z > 50.0 || self.wheel_vel_cmd.w > 50.0 {
            defmt::warn!("Wheel vel cmd too high: {}, {}, {}, {}", self.wheel_vel_cmd.x, self.wheel_vel_cmd.y, self.wheel_vel_cmd.z, self.wheel_vel_cmd.w);
            self.wheel_vel_cmd = Vector4f::default();
            self.wheel_torque_cmd = Vector4f::default();
        }
        if self.wheel_torque_cmd.x > 1.0 || self.wheel_torque_cmd.y > 1.0 || self.wheel_torque_cmd.z > 1.0 || self.wheel_torque_cmd.w > 1.0 {
            defmt::warn!("Wheel torque cmd too high: {}, {}, {}, {}", self.wheel_torque_cmd.x, self.wheel_torque_cmd.y, self.wheel_torque_cmd.z, self.wheel_torque_cmd.w);
            self.wheel_vel_cmd = Vector4f::default();
            self.wheel_torque_cmd = Vector4f::default();
        }

        // Copy values to telemetry
        self.debug_telemetry.imu_gyro[2] = gyro_theta_meas;
        self.debug_telemetry.body_cmd.copy_from_slice(pose_cmd.as_slice());
        self.debug_telemetry.kf_body_pose_estimate.copy_from_slice(&state_estimate.as_slice()[0..3]);
        self.debug_telemetry.kf_body_twist_estimate.copy_from_slice(&state_estimate.as_slice()[3..6]);
        self.debug_telemetry.body_twist_u.copy_from_slice(global_twist_cmd.as_slice());
        self.debug_telemetry.body_accel_u.copy_from_slice(global_accel_cmd.as_slice());
        self.debug_telemetry.wheel_velocity_u.copy_from_slice(self.wheel_vel_cmd.as_slice());
        self.debug_telemetry.wheel_torque_u.copy_from_slice(self.wheel_torque_cmd.as_slice());

        // TODO: kalman filter predict next state
        // // Use control law adjusted value to predict the next cycle's state.
        // self.body_vel_filter.predict(&wheel_vel_output);

        self.robot_model.kf_predict(global_accel_cmd);
    }

    pub fn get_wheel_velocities(&self) -> Vector4f {
        self.wheel_vel_cmd
    }

    pub fn get_wheel_torques(&self) -> Vector4f {
        self.wheel_torque_cmd
    }

    pub fn get_control_debug_telem(&self) -> ExtendedTelemetry {
        self.debug_telemetry
    }
}

impl ParameterInterface for BodyPoseController {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool {
        return self.has_name(param_cmd.parameter_name);
    }

    fn has_name(&self, param_name: ParameterName::Type) -> bool {
        return match param_name {
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
            match param_cmd.parameter_name {
                _ => {
                    defmt::debug!("unimplemented key read in RobotController");
                    reply_cmd.command_code = PCC_NACK_INVALID_NAME;
                    return Err(reply_cmd);
                }
            }
        } else if param_cmd.command_code == PCC_WRITE {
            match param_cmd.parameter_name {
                _ => {
                    defmt::debug!("unimplemented key write in RobotController");
                    reply_cmd.command_code = PCC_NACK_INVALID_NAME;
                    return Err(reply_cmd);
                }
            }
        }

        return Ok(reply_cmd);
    }
}
