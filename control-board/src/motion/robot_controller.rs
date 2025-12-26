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
use ateam_controls::{Vector3f, Vector4f, RigidBodyState};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::robot_model::{transform_frame_global2robot_accel, transform_frame_global2robot_twist, transform_frame_robot2global_twist};
use ateam_controls::robot_physical_params::*;
use ateam_controls::bangbang_trajectory::BangBangTraj3D;
use embassy_stm32::pac::adc::vals::Exten;
use embassy_time::{Duration, Instant};
use nalgebra::{SVector, Vector3, Vector4, Vector5};

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
            robot_model: ateam_controls::robot_model::RobotModel::new(WHEEL_ANGLE_ALPHA, WHEEL_ANGLE_BETA, WHEEL_DISTANCE, WHEEL_RADIUS, BODY_MASS, BODY_MOMENT_Z),
            wheel_vel_cmd: Vector4f::default(),
            wheel_torque_cmd: Vector4f::default(),
            debug_telemetry: Default::default(),
        }
    }

    pub fn control_update(
        &mut self,
        state_setpoint: Vector3f,
        loop_period: Duration,
        vision_pose_meas: Vector3f,
        vision_pose_meas_instant: Instant,
        wheel_vel_meas: Vector4f,
        wheel_torque_meas: Vector4f,
        gyro_theta_meas: f32,
        controls_enabled: bool,
    ) {
        // Assign telemetry data
        // TODO pass all of the gyro data up, not just theta
        self.debug_telemetry.imu_gyro[2] = gyro_theta_meas;

        // TODO: update extended packet with commanded position
        // self.debug_telemetry
        //     .commanded_body_velocity
        //     .copy_from_slice(body_vel_setpoint.as_slice());

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

        // Use raw wheel readings and vision measurements for state estimate until kalman filter is implemented
        let robot_twist = self.robot_model.wheel_velocities_to_twist(wheel_vel_meas);
        let global_twist = transform_frame_robot2global_twist(vision_pose_meas, robot_twist);

        let state_estimate = RigidBodyState { 
            pose: vision_pose_meas,
            twist: global_twist,
        };

        // Calculate the optimal trajectory to the setpoint
        let traj = ateam_controls::bangbang_trajectory::compute_optimal_bangbang_traj_3d(
            state_estimate, state_setpoint
        );
        // Calculate the acceleration needed to achieve the trajectory right now
        let global_accel_cmd = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_accel_at_t(traj, 0.0);
        // Calculate the velocity that should be achieved at the next time step after applying this acceleration
        let next_body_state = ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_state_at_t(traj, state_estimate, 0.0, loop_period.as_micros() as f32 * 1e-6);

        // TODO: output in telemetery
        // self.debug_telemetry
        //     .body_velocity_u
        //     .copy_from_slice(body_vel_output.as_slice());

        // TODO: do any hard clamping? although I think it should already be clamped

        let robot_accel_cmd = transform_frame_global2robot_accel(state_estimate.pose, global_accel_cmd);
        self.wheel_torque_cmd = self.robot_model.accel_to_wheel_torques(robot_accel_cmd);
        let robot_twist_next_body_state = transform_frame_global2robot_twist(next_body_state.pose, next_body_state.twist);
        self.wheel_vel_cmd = self.robot_model.twist_to_wheel_velocities(robot_twist_next_body_state);

        // output in telemetry
        self.debug_telemetry.wheel_velocity_u.copy_from_slice(Vector4::from(self.wheel_vel_cmd).as_slice());

        // TODO: kalman filter update
        // // Use control law adjusted value to predict the next cycle's state.
        // self.body_vel_filter.predict(&wheel_vel_output);

        // TODO: needed?
        // Save command state.
        // if controls_enabled {
        //     self.cmd_wheel_velocities = wheel_vel_output;
        // } else {
        //     self.cmd_wheel_velocities = self.robot_model.robot_vel_to_wheel_vel(&body_vel_setpoint);
        //     self.debug_telemetry
        //         .wheel_velocity_u
        //         .copy_from_slice(wheel_vel_output.as_slice());
        // }
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
