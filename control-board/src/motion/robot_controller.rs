use ateam_common_packets::bindings::{
    ParameterCommandCode::*,
    ParameterName,
    ParameterDataFormat::{PID_LIMITED_INTEGRAL_F32, VEC3_F32, VEC4_F32},
    ParameterName::{VEL_PID_X, RC_BODY_VEL_LIMIT, RC_BODY_ACC_LIMIT, VEL_PID_Y, ANGULAR_VEL_PID_Z, VEL_CGKF_ENCODER_NOISE, VEL_CGKF_PROCESS_NOISE, VEL_CGKF_GYRO_NOISE, VEL_CGFK_INITIAL_COVARIANCE, VEL_CGKF_K_MATRIX, RC_WHEEL_ACC_LIMIT}};
use ateam_common_packets::radio::get_motor_response_motion_packet;
use nalgebra::{SVector, Vector3, Vector4, Vector5};
use crate::parameter_interface::ParameterInterface;

use super::constant_gain_kalman_filter::CgKalmanFilter;
use super::pid::PidController;
use super::robot_model::RobotModel;

use super::params::body_vel_filter_params::{
    KF_NUM_STATES, KF_NUM_CONTROL_INPUTS, KF_NUM_OBSERVATIONS, 
    STATE_TRANSITION, CONTROL_INPUT, OBSERVATION_MODEL, PROCESS_COV, INIT_ESTIMATE_COV, KALMAN_GAIN,
};

use super::params::body_vel_pid_params::{
    PID_GAIN,
    BODY_VEL_LIM, BODY_ACC_LIM, BODY_DEACC_LIM, WHEEL_ACC_LIM
};

use ateam_common_packets::bindings::{
    ControlDebugTelemetry,
    ParameterCommand
};

// TODO find some general numeric type trait(s) for D
// Clamp the vector, but keep the direction consistent.
pub fn clamp_vector_keep_dir<const D: usize>(vec: &SVector<f32, D>, limits_abs: &SVector<f32, D>) -> SVector<f32, D> {
    // Applies the clamping in a sign-less way by getting the scale of the vector compared to absolute value clamp region.
    let scales: SVector<f32, D> = vec.abs().component_div(limits_abs);
    // To maintain direction, do a scalar divison of the max of the scales. Limit to make sure to only scale down.
    vec / f32::max(1.0, scales.max())
}

pub struct BodyVelocityController<'a> {
    loop_dt_s: f32,
    robot_model: RobotModel,
    body_vel_filter: CgKalmanFilter<'a, 3, 4, 5>,
    body_vel_controller: PidController<3>,
    body_velocity_limit: Vector3<f32>,
    body_acceleration_limit: Vector3<f32>,
    body_deceleration_limit: Vector3<f32>,
    wheel_acceleration_limits: Vector4<f32>,
    prev_output: Vector3<f32>,
    cmd_wheel_velocities: Vector4<f32>,
    debug_telemetry: ControlDebugTelemetry,
}

impl<'a> BodyVelocityController<'a> {
    pub fn new_from_global_params(loop_dt_s: f32, robot_model: RobotModel) -> BodyVelocityController<'a> {
        let body_vel_filter: CgKalmanFilter<KF_NUM_STATES, KF_NUM_CONTROL_INPUTS, KF_NUM_OBSERVATIONS> = 
            CgKalmanFilter::new(&STATE_TRANSITION, &CONTROL_INPUT, &OBSERVATION_MODEL, &PROCESS_COV, &KALMAN_GAIN, &INIT_ESTIMATE_COV);

        let body_vel_controller: PidController<KF_NUM_STATES> =
            PidController::from_gains_matrix(&PID_GAIN);

        let mut bvc = BodyVelocityController {
            loop_dt_s: loop_dt_s,
            robot_model: robot_model,
            body_vel_filter: body_vel_filter,
            body_vel_controller: body_vel_controller,
            body_velocity_limit: Vector3::zeros(),
            body_acceleration_limit: Vector3::zeros(),
            body_deceleration_limit: Vector3::zeros(),
            wheel_acceleration_limits: Vector4::zeros(),
            prev_output: Vector3::zeros(),
            cmd_wheel_velocities: Vector4::zeros(),
            debug_telemetry: ControlDebugTelemetry { 
                motor_fl: get_motor_response_motion_packet(),
                motor_bl: get_motor_response_motion_packet(),
                motor_br: get_motor_response_motion_packet(),
                motor_fr: get_motor_response_motion_packet(),
                imu_gyro: [0.0, 0.0, 0.0],
                imu_accel: [0.0, 0.0, 0.0],
                commanded_body_velocity: [0.0, 0.0, 0.0],
                clamped_commanded_body_velocity: [0.0, 0.0, 0.0],
                cgkf_body_velocity_state_estimate: [0.0, 0.0, 0.0],
                body_velocity_u: [0.0, 0.0, 0.0],
                wheel_velocity_u: [0.0, 0.0, 0.0, 0.0],
                wheel_velocity_clamped_u: [0.0, 0.0, 0.0, 0.0] 
            }
        };

        bvc.body_velocity_limit.copy_from(&BODY_VEL_LIM);
        bvc.body_acceleration_limit.copy_from(&BODY_ACC_LIM);
        bvc.body_deceleration_limit.copy_from(&BODY_DEACC_LIM);
        bvc.wheel_acceleration_limits.copy_from(&WHEEL_ACC_LIM);

        bvc
    }

    pub fn new(loop_dt_s: f32,
            robot_model: RobotModel,
            body_vel_filter: CgKalmanFilter<'a, 3, 4, 5>,
            pid_controller: PidController<3>,
            bv_limit: Vector3<f32>,
            ba_limit: Vector3<f32>,
            bda_limit: Vector3<f32>,
            wa_limit: Vector4<f32>
    ) -> BodyVelocityController<'a> {
        BodyVelocityController {
            loop_dt_s: loop_dt_s,
            robot_model: robot_model,
            body_vel_filter: body_vel_filter,
            body_vel_controller: pid_controller,
            body_velocity_limit: bv_limit,
            body_acceleration_limit: ba_limit,
            body_deceleration_limit: bda_limit,
            wheel_acceleration_limits: wa_limit,
            prev_output: Vector3::zeros(),
            cmd_wheel_velocities: Vector4::zeros(),
            debug_telemetry: ControlDebugTelemetry { 
                motor_fl: get_motor_response_motion_packet(),
                motor_bl: get_motor_response_motion_packet(),
                motor_br: get_motor_response_motion_packet(),
                motor_fr: get_motor_response_motion_packet(),
                imu_gyro: [0.0, 0.0, 0.0],
                imu_accel: [0.0, 0.0, 0.0],
                commanded_body_velocity: [0.0, 0.0, 0.0],
                clamped_commanded_body_velocity: [0.0, 0.0, 0.0],
                cgkf_body_velocity_state_estimate: [0.0, 0.0, 0.0],
                body_velocity_u: [0.0, 0.0, 0.0],
                wheel_velocity_u: [0.0, 0.0, 0.0, 0.0],
                wheel_velocity_clamped_u: [0.0, 0.0, 0.0, 0.0] 
            }
        }
    }

    pub fn control_update(&mut self, body_vel_setpoint: &Vector3<f32>, wheel_velocities: &Vector4<f32>, _wheel_torques: &Vector4<f32>, gyro_theta: f32, controls_enabled: bool) {
        // Assign telemetry data
        // TODO pass all of the gyro data up, not just theta
        self.debug_telemetry.imu_gyro[2] = gyro_theta;
        
        self.debug_telemetry.commanded_body_velocity.copy_from_slice(body_vel_setpoint.as_slice());

        let measurement: Vector5<f32> = Vector5::new(wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3], gyro_theta);

        // Update measurements process observation input into CGKF.
        self.body_vel_filter.update(&measurement);

        // Read the current body velocity state estimate from the CGKF.
        let mut body_vel_estimate = self.body_vel_filter.get_state();

        // Deadzone the velocity estimate
        if libm::fabsf(body_vel_estimate[0]) < 0.05 {
            body_vel_estimate[0] = 0.0;
        }

        if libm::fabsf(body_vel_estimate[1]) < 0.05 {
            body_vel_estimate[1] = 0.0;
        }

        if libm::fabsf(body_vel_estimate[2]) < 0.05 {
            body_vel_estimate[2] = 0.0;
        }

        self.debug_telemetry.cgkf_body_velocity_state_estimate.copy_from_slice(body_vel_estimate.as_slice());

        // Apply control policy.
        let body_vel_control_pid = self.body_vel_controller.calculate(&body_vel_setpoint, &body_vel_estimate, self.loop_dt_s);

        // Add the commanded setpoint as a feedforward component.
        let body_vel_output = body_vel_control_pid + body_vel_setpoint;
        // let body_vel_output = body_vel_setpoint;
        
        self.debug_telemetry.body_velocity_u.copy_from_slice(body_vel_output.as_slice());

        // Determine commanded body acceleration based on previous control output, and clamp and maintain the direction of acceleration.
        // NOTE: Using previous control output instead of estimate so that collision disturbances would not impact.
        let body_acc_output = (body_vel_output - self.prev_output) / self.loop_dt_s;
        // Make a copy of acceleration to swap out with decel parts.
        let mut temp_accel_decel_holder: Vector3<f32> = Vector3::zeros();
        temp_accel_decel_holder.copy_from_slice(self.body_acceleration_limit.as_slice());
        // Check if each part is decel (sign of acceleration != est velocity)
        if (body_acc_output[0] > 0.0) != (body_vel_estimate[0] > 0.0) {
            temp_accel_decel_holder[0] = self.body_deceleration_limit[0];
        }

        if (body_acc_output[1] > 0.0) != (body_vel_estimate[1] > 0.0) {
            temp_accel_decel_holder[1] = self.body_deceleration_limit[1];
        }

        if (body_acc_output[2] > 0.0) != (body_vel_estimate[2] > 0.0) {
            temp_accel_decel_holder[2] = self.body_deceleration_limit[2];
        }

        let body_acc_output_clamp = clamp_vector_keep_dir(&body_acc_output, &temp_accel_decel_holder);

        // Convert back to body velocity
        let body_vel_output_acc_clamp = self.prev_output + (body_acc_output_clamp * self.loop_dt_s);

        // Clamp and maintain direction of control body velocity.
        let body_vel_output_full_clamp = clamp_vector_keep_dir(&body_vel_output_acc_clamp, &self.body_velocity_limit);
        self.prev_output.copy_from_slice(body_vel_output_full_clamp.as_slice());
        self.debug_telemetry.clamped_commanded_body_velocity.copy_from_slice(body_vel_output_full_clamp.as_slice());

        // Transform body velocity commands into the wheel velocity domain.
        let wheel_vel_output = self.robot_model.robot_vel_to_wheel_vel(&body_vel_output_full_clamp);
        self.debug_telemetry.wheel_velocity_u.copy_from_slice(wheel_vel_output.as_slice());

        // Use control law adjusted value to predict the next cycle's state.
        self.body_vel_filter.predict(&wheel_vel_output);

        // determine commanded wheel accleration, and clamp-scale the the control input
        // TODO a linear clamp after the non-linear domain transformation is probably locally valid
        // and globally invalid. Investiage this later. If problems are suspected, disable this section
        // and lower the body acc limit (maybe something anatgonist based on 45/30 deg wheel angles?)
        // TODO cross check in the future against wheel angle plots and analysis
        //let wheel_acc_setpoint = (wheel_vel_output - self.cmd_wheel_velocities) / self.loop_dt_s;
        //let wheel_acc_setpoint_clamp = clamp_vector_keep_dir(&wheel_acc_setpoint, &WHEEL_ACC_LIM);
        //let wheel_vel_output_clamp = self.cmd_wheel_velocities + (wheel_acc_setpoint_clamp * self.loop_dt_s);
        //self.debug_telemetry.wheel_velocity_clamped_u.copy_from_slice(wheel_vel_output_clamp.as_slice());

        // Save command state.
        if controls_enabled {
            self.cmd_wheel_velocities = wheel_vel_output;
        } else {
            self.cmd_wheel_velocities = self.robot_model.robot_vel_to_wheel_vel(&body_vel_setpoint);
            self.debug_telemetry.wheel_velocity_u.copy_from_slice(wheel_vel_output.as_slice());
        }
    }

    pub fn get_wheel_velocities(&self) -> Vector4<f32> {
        self.cmd_wheel_velocities
    }

    pub fn get_control_debug_telem(&self) -> ControlDebugTelemetry {
        self.debug_telemetry
    }
}

impl<'a> ParameterInterface for BodyVelocityController<'a> {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool {
        return self.has_name(param_cmd.parameter_name);
    }

    fn has_name(&self, param_name: ParameterName::Type) -> bool {
        return match param_name {
            RC_BODY_VEL_LIMIT | RC_BODY_ACC_LIMIT | RC_WHEEL_ACC_LIMIT => true,
            VEL_PID_X | VEL_PID_Y | ANGULAR_VEL_PID_Z => true,
            VEL_CGKF_ENCODER_NOISE | VEL_CGKF_PROCESS_NOISE | VEL_CGKF_GYRO_NOISE | VEL_CGFK_INITIAL_COVARIANCE | VEL_CGKF_K_MATRIX => true,
            _ => false,
        }
    }


    fn apply_command(&mut self, param_cmd: &ParameterCommand) -> Result<ParameterCommand, ParameterCommand> {
        let mut reply_cmd = param_cmd.clone();

        // if we haven't been given an actionable command code, ignore the call
        if !(param_cmd.command_code == PCC_READ || param_cmd.command_code == PCC_WRITE) {
            defmt::warn!("asked to apply a command with out and actional command code");
            return Err(reply_cmd);
        }

        // if we've been asked to apply a command we don't have a key for it
        // error out
        if !self.has_name(param_cmd.parameter_name) {
            defmt::warn!("asked to apply a command with a parameter name not managed by this module");
            reply_cmd.command_code = PCC_NACK_INVALID_NAME;
            return Err(*param_cmd);
        }

        if param_cmd.command_code == PCC_READ {
            match param_cmd.parameter_name {
                VEL_PID_X => {
                    // set the type
                    reply_cmd.data_format = PID_LIMITED_INTEGRAL_F32;

                    // readback the data
                    let current_pid_gain = self.body_vel_controller.get_gain();

                    // can't slice copy b/c backing storage is column-major
                    // so a row slice isn't contiguous in backing memory and
                    // therefore you can't do a slice copy
                    unsafe {
                        reply_cmd.data.pidii_f32[0] = current_pid_gain.row(0)[0];
                        reply_cmd.data.pidii_f32[1] = current_pid_gain.row(0)[1];
                        reply_cmd.data.pidii_f32[2] = current_pid_gain.row(0)[2];
                        reply_cmd.data.pidii_f32[3] = current_pid_gain.row(0)[3];
                        reply_cmd.data.pidii_f32[4] = current_pid_gain.row(0)[4];
                    }

                    reply_cmd.command_code = PCC_ACK;
                },
                VEL_PID_Y => {
                    reply_cmd.data_format = PID_LIMITED_INTEGRAL_F32;

                    let current_pid_gain = self.body_vel_controller.get_gain();
                    unsafe {
                        reply_cmd.data.pidii_f32[0] = current_pid_gain.row(1)[0];
                        reply_cmd.data.pidii_f32[1] = current_pid_gain.row(1)[1];
                        reply_cmd.data.pidii_f32[2] = current_pid_gain.row(1)[2];
                        reply_cmd.data.pidii_f32[3] = current_pid_gain.row(1)[3];
                        reply_cmd.data.pidii_f32[4] = current_pid_gain.row(1)[4];
                    }

                    reply_cmd.command_code = PCC_ACK;
                },
                ANGULAR_VEL_PID_Z => {
                    reply_cmd.data_format = PID_LIMITED_INTEGRAL_F32;
                    
                    let current_pid_gain = self.body_vel_controller.get_gain();
                    unsafe {
                        reply_cmd.data.pidii_f32[0] = current_pid_gain.row(2)[0];
                        reply_cmd.data.pidii_f32[1] = current_pid_gain.row(2)[1];
                        reply_cmd.data.pidii_f32[2] = current_pid_gain.row(2)[2];
                        reply_cmd.data.pidii_f32[3] = current_pid_gain.row(2)[3];
                        reply_cmd.data.pidii_f32[4] = current_pid_gain.row(2)[4];
                    }

                    reply_cmd.command_code = PCC_ACK;
                },
                // VEL_CGKF_ENCODER_NOISE => {

                // },
                // VEL_CGKF_GYRO_NOISE => {
                    
                // }
                // VEL_CGKF_PROCESS_NOISE => {

                // },
                // VEL_CGFK_INITIAL_COVARIANCE => {

                // },
                // VEL_CGKF_K_MATRIX => {

                // },
                RC_BODY_VEL_LIMIT => {
                    // set the type
                    reply_cmd.data_format = VEC3_F32;
                    // read back the data
                    unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.body_velocity_limit.as_slice()); }

                    reply_cmd.command_code = PCC_ACK;
                },
                RC_BODY_ACC_LIMIT => {
                    reply_cmd.data_format = VEC3_F32;
                    unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.body_acceleration_limit.as_slice()); }
                    reply_cmd.command_code = PCC_ACK;
                },
                RC_WHEEL_ACC_LIMIT => {
                    reply_cmd.data_format = VEC4_F32;
                    unsafe { reply_cmd.data.vec4_f32.copy_from_slice(self.wheel_acceleration_limits.as_slice()); }
                    reply_cmd.command_code = PCC_ACK;
                },
                _ => {
                    defmt::debug!("unimplemented key read in RobotController");
                    reply_cmd.command_code = PCC_NACK_INVALID_NAME;
                    return Err(reply_cmd);
                }
            }
        } else if param_cmd.command_code == PCC_WRITE {
            match param_cmd.parameter_name {
                VEL_PID_X => {
                    if param_cmd.data_format == PID_LIMITED_INTEGRAL_F32 {
                        // read data into matrix, modify the matrix row, then write the whole thing back
                        let mut current_pid_gain = self.body_vel_controller.get_gain();
                        current_pid_gain.row_mut(0).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
                        self.body_vel_controller.set_gain(current_pid_gain);

                        
                        // can't slice copy b/c backing storage is column-major
                        // so a row slice isn't contiguous in backing memory and
                        // therefore you can't do a slice copy
                        let updated_pid_gain = self.body_vel_controller.get_gain();
                        unsafe {
                            reply_cmd.data.pidii_f32[0] = updated_pid_gain.row(0)[0];
                            reply_cmd.data.pidii_f32[1] = updated_pid_gain.row(0)[1];
                            reply_cmd.data.pidii_f32[2] = updated_pid_gain.row(0)[2];
                            reply_cmd.data.pidii_f32[3] = updated_pid_gain.row(0)[3];
                            reply_cmd.data.pidii_f32[4] = updated_pid_gain.row(0)[4];
                        }

                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                VEL_PID_Y => {
                    if param_cmd.data_format == PID_LIMITED_INTEGRAL_F32 {
                        let mut current_pid_gain = self.body_vel_controller.get_gain();
                        current_pid_gain.row_mut(1).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
                        self.body_vel_controller.set_gain(current_pid_gain);

                        let updated_pid_gain = self.body_vel_controller.get_gain();
                        unsafe {
                            reply_cmd.data.pidii_f32[0] = updated_pid_gain.row(1)[0];
                            reply_cmd.data.pidii_f32[1] = updated_pid_gain.row(1)[1];
                            reply_cmd.data.pidii_f32[2] = updated_pid_gain.row(1)[2];
                            reply_cmd.data.pidii_f32[3] = updated_pid_gain.row(1)[3];
                            reply_cmd.data.pidii_f32[4] = updated_pid_gain.row(1)[4];
                        }

                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                ANGULAR_VEL_PID_Z => {
                    if param_cmd.data_format == PID_LIMITED_INTEGRAL_F32 {
                        let mut current_pid_gain = self.body_vel_controller.get_gain();
                        current_pid_gain.row_mut(2).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
                        self.body_vel_controller.set_gain(current_pid_gain);

                        let updated_pid_gain = self.body_vel_controller.get_gain();
                        unsafe {
                            reply_cmd.data.pidii_f32[0] = updated_pid_gain.row(2)[0];
                            reply_cmd.data.pidii_f32[1] = updated_pid_gain.row(2)[1];
                            reply_cmd.data.pidii_f32[2] = updated_pid_gain.row(2)[2];
                            reply_cmd.data.pidii_f32[3] = updated_pid_gain.row(2)[3];
                            reply_cmd.data.pidii_f32[4] = updated_pid_gain.row(2)[4];
                        }

                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                // VEL_CGKF_ENCODER_NOISE => {

                // },
                // VEL_CGKF_GYRO_NOISE => {
                    
                // }
                // VEL_CGKF_PROCESS_NOISE => {

                // },
                // VEL_CGFK_INITIAL_COVARIANCE => {

                // },
                // VEL_CGKF_K_MATRIX => {

                // },
                RC_BODY_VEL_LIMIT => {
                    if param_cmd.data_format == VEC3_F32 {
                        // write the new data, then read it back into the reply
                        self.body_velocity_limit.as_mut_slice().copy_from_slice(unsafe { &param_cmd.data.vec3_f32 });
                        unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.body_velocity_limit.as_slice()); }
    
                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                RC_BODY_ACC_LIMIT => {
                    if param_cmd.data_format == VEC3_F32 {
                        // write the new data, then read it back into the reply
                        self.body_acceleration_limit.as_mut_slice().copy_from_slice(unsafe { &param_cmd.data.vec3_f32 });
                        unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.body_acceleration_limit.as_slice()); }
    
                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                RC_WHEEL_ACC_LIMIT => {
                    if param_cmd.data_format == VEC4_F32 {
                        // write the new data, then read it back into the reply
                        self.wheel_acceleration_limits.as_mut_slice().copy_from_slice(unsafe { &param_cmd.data.vec4_f32 });
                        unsafe { reply_cmd.data.vec4_f32.copy_from_slice(self.wheel_acceleration_limits.as_slice()); }
    
                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
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
