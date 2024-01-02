use ateam_common_packets::bindings_radio::{ParameterCommandCode::*, ParameterName};
use ateam_common_packets::bindings_radio::ParameterDataFormat::{PID_LIMITED_INTEGRAL_F32, VEC3_F32, VEC4_F32, F32};
use ateam_common_packets::bindings_radio::ParameterName::{VEL_PID_X, RC_BODY_VEL_LIMIT, RC_BODY_ACC_LIMIT, VEL_PID_Y, ANGULAR_VEL_PID_Z, VEL_CGKF_ENCODER_NOISE, VEL_CGKF_PROCESS_NOISE, VEL_CGKF_GYRO_NOISE, VEL_CGFK_INITIAL_COVARIANCE, VEL_CGKF_K_MATRIX, RC_WHEEL_ACC_LIMIT};
use nalgebra::{SVector, Vector3, Vector4, Vector5};

use crate::parameter_interface::ParameterInterface;

use super::constant_gain_kalman_filter::CgKalmanFilter;
use super::pid::PidController;
use super::robot_model::RobotModel;

use super::params::body_vel_filter_params::{
    F, B, H, Q, R, P, K,
    KfNumStates, KfNumControlInputs, KfNumObservations,
};

use super::params::body_vel_pid_params::{
    K_pid,
    BODY_VEL_LIM, BODY_ACC_LIM, WHEEL_ACC_LIM
};

use ateam_common_packets::bindings_radio::{
    ControlDebugTelemetry,
    MotorDebugTelemetry, ParameterCommand
};

// TODO find some general numeric type trait(s) for D
pub fn clamp_scale_vector<const D: usize>(vec: &SVector<f32, D>, cabs: &SVector<f32, D>) -> SVector<f32, D> {
    let scales: SVector<f32, D> = vec.abs().component_div(cabs).map(|val| f32::max(1.0, val));
    vec.component_div(&scales)
}

pub struct BodyVelocityController<'a> {
    loop_dt_s: f32,
    robot_model: RobotModel,
    body_vel_filter: CgKalmanFilter<'a, 3, 4, 5>,
    body_vel_controller: PidController<3>,
    body_velocity_limit: Vector3<f32>,
    body_acceleration_limit: Vector3<f32>,
    wheel_acceleration_limits: Vector4<f32>,
    prev_setpoint: Vector3<f32>,
    prev_wheel_u: Vector4<f32>,
    cmd_wheel_velocities: Vector4<f32>,
    debug_telemetry: ControlDebugTelemetry,
}

impl<'a> BodyVelocityController<'a> {
    pub fn new_from_global_params(loop_dt_s: f32, robot_model: RobotModel) -> BodyVelocityController<'a> {
        let body_vel_filter: CgKalmanFilter<KfNumStates, KfNumControlInputs, KfNumObservations> = 
                CgKalmanFilter::new(&F, &B, &H, &Q, &R, &P, &K);

        let body_vel_controller: PidController<KfNumStates> =
                PidController::from_gains_matrix(&K_pid);

        let mut bvc = BodyVelocityController {
            loop_dt_s: loop_dt_s,
            robot_model: robot_model,
            body_vel_filter: body_vel_filter,
            body_vel_controller: body_vel_controller,
            body_velocity_limit: Vector3::zeros(),
            body_acceleration_limit: Vector3::zeros(),
            wheel_acceleration_limits: Vector4::zeros(),
            prev_setpoint: Vector3::zeros(),
            prev_wheel_u: Vector4::zeros(),
            cmd_wheel_velocities: Vector4::zeros(),
            debug_telemetry: ControlDebugTelemetry { 
                motor_fl: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
                motor_fr: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
                motor_br: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
                motor_bl: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
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
        bvc.wheel_acceleration_limits.copy_from(&WHEEL_ACC_LIM);

        bvc
    }

    pub fn new(loop_dt_s: f32,
            robot_model: RobotModel,
            body_vel_filter: CgKalmanFilter<'a, 3, 4, 5>,
            pid_controller: PidController<3>,
            bv_limit: Vector3<f32>,
            ba_limit: Vector3<f32>,
            wa_limit: Vector4<f32>
    ) -> BodyVelocityController<'a> {
        BodyVelocityController {
            loop_dt_s: loop_dt_s,
            robot_model: robot_model,
            body_vel_filter: body_vel_filter,
            body_vel_controller: pid_controller,
            body_velocity_limit: bv_limit,
            body_acceleration_limit: ba_limit,
            wheel_acceleration_limits: wa_limit,
            prev_setpoint: Vector3::zeros(),
            prev_wheel_u: Vector4::zeros(),
            cmd_wheel_velocities: Vector4::zeros(),
            debug_telemetry: ControlDebugTelemetry { 
                motor_fl: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
                motor_fr: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
                motor_br: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
                motor_bl: MotorDebugTelemetry { 
                    wheel_setpoint: 0.0,
                    wheel_velocity: 0.0,
                    wheel_torque: 0.0
                },
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

    pub fn control_update(&mut self, setpoint: &Vector3<f32>, wheel_velocities: &Vector4<f32>, wheel_torques: &Vector4<f32>, gyro_theta: f32) {
        self.debug_telemetry.motor_fr.wheel_torque = wheel_torques[1];
        self.debug_telemetry.motor_fl.wheel_torque = wheel_torques[0];
        self.debug_telemetry.motor_bl.wheel_torque = wheel_torques[3];
        self.debug_telemetry.motor_br.wheel_torque = wheel_torques[2];

        // construct the observation vector the KF expects
        let z: Vector5<f32> = Vector5::new(wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3], gyro_theta);
        self.control_update_z(setpoint, z);
    }

    pub fn control_update_z(&mut self, setpoint: &Vector3<f32>, z: Vector5<f32>) {
        // TODO there are a few discrete time intergrals and derivatives in here
        // these should probably be genericized/templated some how



        // clamp/scale setpoint vel
        let setpoint = clamp_scale_vector(setpoint, &BODY_VEL_LIM);
        self.debug_telemetry.commanded_body_velocity.copy_from_slice(setpoint.as_slice());

        // Firmware does FR, FL, BL, BR ordering like graph quadrants
        // Software does FL, FR, BR, BL ordering "clockwise"
        // we'll do the mapping here for now
        // FIX ME
        self.debug_telemetry.imu_gyro[2] = z.a;
        self.debug_telemetry.motor_fr.wheel_velocity = z[1];
        self.debug_telemetry.motor_fl.wheel_velocity = z[0];
        self.debug_telemetry.motor_bl.wheel_velocity = z[3];
        self.debug_telemetry.motor_br.wheel_velocity = z[2];

        // determine commanded body accleration, and clamp-scale the the control input
        let sp_body_acc = (setpoint - self.prev_setpoint) / self.loop_dt_s;
        let sp_body_acc_limited = clamp_scale_vector(&sp_body_acc, &BODY_ACC_LIM);
        let setpoint_limited = self.prev_setpoint + (sp_body_acc_limited * self.loop_dt_s);
        self.prev_setpoint = setpoint_limited;
        self.debug_telemetry.clamped_commanded_body_velocity.copy_from_slice(setpoint_limited.as_slice());

        // get latest wheel vals and gyro (observe the state)
        // process observation input into CGKF
        self.body_vel_filter.update(&z);

        // read the current body velocity state estimate from the CGKF
        let y = self.body_vel_filter.get_state();
        self.debug_telemetry.cgkf_body_velocity_state_estimate.copy_from_slice(y.as_slice());

        // apply control policy
        self.body_vel_controller.calculate(setpoint_limited, y, self.loop_dt_s);
        let u = self.body_vel_controller.get_u();
        self.debug_telemetry.body_velocity_u.copy_from_slice(u.as_slice());

        // transform body velocity commands into the wheel velocity domain
        let wheel_u = self.robot_model.robot_vel_to_wheel_vel(u);
        self.debug_telemetry.wheel_velocity_u.copy_from_slice(wheel_u.as_slice());

        // use control law adjusted value to predict the next cycle's state
        // call CGKF.predict
        self.body_vel_filter.predict(&wheel_u);

        // determine commanded wheel accleration, and clamp-scale the the control input
        // TODO a linear clamp after the non-linear domain transformation is probably locally valid
        // and globally invalid. Investiage this later. If problems are suspected, disable this section
        // and lower the body acc limit (maybe something anatgonist based on 45/30 deg wheel angles?)
        // TODO cross check in the future against wheel angle plots and analysis
        let sp_wheel_acc = (wheel_u - self.prev_wheel_u) / self.loop_dt_s;
        let sp_wheel_acc_limited = clamp_scale_vector(&sp_wheel_acc, &WHEEL_ACC_LIM);
        let wheel_u_limited = self.prev_wheel_u + (sp_wheel_acc_limited * self.loop_dt_s);
        self.debug_telemetry.wheel_velocity_clamped_u.copy_from_slice(wheel_u_limited.as_slice());
        
        // update previous u
        self.prev_wheel_u = wheel_u_limited;

        // save command state
        // self.cmd_wheel_velocities = wheel_u;
        self.cmd_wheel_velocities = wheel_u_limited;

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
                    #[allow(non_snake_case)] // K is the mathematical symbol for this matrix
                    let current_K = self.body_vel_controller.get_K();

                    // can't slice copy b/c backing storage is column-major
                    // so a row slice isn't contiguous in backing memory and
                    // therefore you can't do a slice copy
                    unsafe {
                        reply_cmd.data.pidii_f32[0] = current_K.row(0)[0];
                        reply_cmd.data.pidii_f32[1] = current_K.row(0)[1];
                        reply_cmd.data.pidii_f32[2] = current_K.row(0)[2];
                        reply_cmd.data.pidii_f32[3] = current_K.row(0)[3];
                        reply_cmd.data.pidii_f32[4] = current_K.row(0)[4];
                    }

                    reply_cmd.command_code = PCC_ACK;
                },
                VEL_PID_Y => {
                    reply_cmd.data_format = PID_LIMITED_INTEGRAL_F32;

                    #[allow(non_snake_case)]
                    let current_K = self.body_vel_controller.get_K();
                    unsafe {
                        reply_cmd.data.pidii_f32[0] = current_K.row(1)[0];
                        reply_cmd.data.pidii_f32[1] = current_K.row(1)[1];
                        reply_cmd.data.pidii_f32[2] = current_K.row(1)[2];
                        reply_cmd.data.pidii_f32[3] = current_K.row(1)[3];
                        reply_cmd.data.pidii_f32[4] = current_K.row(1)[4];
                    }

                    reply_cmd.command_code = PCC_ACK;
                },
                ANGULAR_VEL_PID_Z => {
                    reply_cmd.data_format = PID_LIMITED_INTEGRAL_F32;
                    
                    #[allow(non_snake_case)]
                    let current_K = self.body_vel_controller.get_K();
                    unsafe {
                        reply_cmd.data.pidii_f32[0] = current_K.row(2)[0];
                        reply_cmd.data.pidii_f32[1] = current_K.row(2)[1];
                        reply_cmd.data.pidii_f32[2] = current_K.row(2)[2];
                        reply_cmd.data.pidii_f32[3] = current_K.row(2)[3];
                        reply_cmd.data.pidii_f32[4] = current_K.row(2)[4];
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
                    unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.wheel_acceleration_limits.as_slice()); }
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
                        #[allow(non_snake_case)]
                        let mut current_K = self.body_vel_controller.get_K();
                        current_K.row_mut(0).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
                        self.body_vel_controller.set_K(current_K);

                        #[allow(non_snake_case)]
                        let updated_K = self.body_vel_controller.get_K();

                        // can't slice copy b/c backing storage is column-major
                        // so a row slice isn't contiguous in backing memory and
                        // therefore you can't do a slice copy
                        unsafe {
                            reply_cmd.data.pidii_f32[0] = updated_K.row(0)[0];
                            reply_cmd.data.pidii_f32[1] = updated_K.row(0)[1];
                            reply_cmd.data.pidii_f32[2] = updated_K.row(0)[2];
                            reply_cmd.data.pidii_f32[3] = updated_K.row(0)[3];
                            reply_cmd.data.pidii_f32[4] = updated_K.row(0)[4];
                        }

                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                VEL_PID_Y => {
                    if param_cmd.data_format == PID_LIMITED_INTEGRAL_F32 {
                        #[allow(non_snake_case)]
                        let mut current_K = self.body_vel_controller.get_K();
                        current_K.row_mut(1).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
                        self.body_vel_controller.set_K(current_K);

                        #[allow(non_snake_case)]
                        let updated_K = self.body_vel_controller.get_K();
                        unsafe {
                            reply_cmd.data.pidii_f32[0] = updated_K.row(1)[0];
                            reply_cmd.data.pidii_f32[1] = updated_K.row(1)[1];
                            reply_cmd.data.pidii_f32[2] = updated_K.row(1)[2];
                            reply_cmd.data.pidii_f32[3] = updated_K.row(1)[3];
                            reply_cmd.data.pidii_f32[4] = updated_K.row(1)[4];
                        }

                        reply_cmd.command_code = PCC_ACK;
                    } else {
                        reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
                        return Err(reply_cmd);
                    }
                },
                ANGULAR_VEL_PID_Z => {
                    if param_cmd.data_format == PID_LIMITED_INTEGRAL_F32 {
                        #[allow(non_snake_case)]
                        let mut current_K = self.body_vel_controller.get_K();
                        current_K.row_mut(2).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
                        self.body_vel_controller.set_K(current_K);

                        #[allow(non_snake_case)]
                        let updated_K = self.body_vel_controller.get_K();
                        unsafe {
                            reply_cmd.data.pidii_f32[0] = updated_K.row(2)[0];
                            reply_cmd.data.pidii_f32[1] = updated_K.row(2)[1];
                            reply_cmd.data.pidii_f32[2] = updated_K.row(2)[2];
                            reply_cmd.data.pidii_f32[3] = updated_K.row(2)[3];
                            reply_cmd.data.pidii_f32[4] = updated_K.row(2)[4];
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
                        self.wheel_acceleration_limits.as_mut_slice().copy_from_slice(unsafe { &param_cmd.data.vec3_f32 });
                        unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.wheel_acceleration_limits.as_slice()); }
    
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

        // match param_cmd.parameter_name {
        //     RC_BODY_VEL_LIMIT => {
        //         if param_cmd.data_format == VEC3_F32 {
        //             // if commanded to write, do the write
        //             if param_cmd.command_code == PCC_WRITE {
        //                 self.body_velocity_limit.as_mut_slice().copy_from_slice(unsafe { &param_cmd.data.vec3_f32 });
        //             }

        //             // write back
        //             unsafe { reply_cmd.data.vec3_f32.copy_from_slice(self.body_velocity_limit.as_slice()); }

        //             reply_cmd.command_code = PCC_ACK;
        //         } else {
        //             reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
        //             return Err(reply_cmd);
        //         }
        //     },
        //     VEL_PID_X => {
        //         if param_cmd.data_format == PID_LIMITED_INTEGRAL_F32 {
        //             defmt::trace!("hit VEL_PID_X update block");
        //             #[allow(non_snake_case)] // K is the mathematical symbol for this matrix
        //             let mut current_K = self.body_vel_controller.get_K();

        //             // if commanded to write, do the write
        //             if param_cmd.command_code == PCC_WRITE {
        //                 defmt::trace!("hit VEL_PID_X write update block");
        //                 current_K.row_mut(0).copy_from_slice(unsafe { &param_cmd.data.pidii_f32 });
        //             }

        //             // can't slice copy b/c backing storage is column-major
        //             // so a row slice isn't contiguous in backing memory and
        //             // therefore you can't do a slice copy
        //             unsafe {
        //                 reply_cmd.data.pidii_f32[0] = current_K.row(0)[0];
        //                 reply_cmd.data.pidii_f32[1] = current_K.row(0)[1];
        //                 reply_cmd.data.pidii_f32[2] = current_K.row(0)[2];
        //                 reply_cmd.data.pidii_f32[3] = current_K.row(0)[3];
        //                 reply_cmd.data.pidii_f32[4] = current_K.row(0)[4];
        //             }

        //             reply_cmd.command_code = PCC_ACK;
        //         } else {
        //             reply_cmd.command_code = PCC_NACK_INVALID_TYPE_FOR_NAME;
        //             return Err(reply_cmd);
        //         }
        //     },
        //     _ => {
        //         defmt::debug!("unimplement key update in RobotController");
        //         reply_cmd.command_code = PCC_NACK_INVALID_NAME;
        //         return Err(reply_cmd);
        //     }
        // }

        return Ok(reply_cmd);
    }
}