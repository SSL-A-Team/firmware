use nalgebra::{SVector, Vector3, Vector4, Vector5};

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
    MotorDebugTelemetry
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
    body_vel_controller: PidController<'a, 3>,
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
            pid_controller: PidController<'a, 3>,
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

    pub fn control_update(&mut self, setpoint: &Vector3<f32>, wheel_velocities: &Vector4<f32>, gyro_theta: f32) {
        // construct the observation vector the KF expects
        let z: Vector5<f32> = Vector5::new(wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3], gyro_theta);
        self.control_update_z(setpoint, z);
    }

    pub fn control_update_z(&mut self, setpoint: &Vector3<f32>, z: Vector5<f32>) {
        // TODO there are a few discrete time intergrals and derivatives in here
        // these should probably be genericized/templated some how

        // Firmware does FR, FL, BL, BR ordering like graph quadrants
        // Software does FL, FR, BR, BL ordering "clockwise"
        // we'll do the mapping here for now
        // FIX ME
        self.debug_telemetry.motor_fl.wheel_velocity = z[1];
        self.debug_telemetry.motor_fr.wheel_velocity = z[0];
        self.debug_telemetry.motor_br.wheel_velocity = z[3];
        self.debug_telemetry.motor_bl.wheel_velocity = z[2];

        self.debug_telemetry.imu_gyro[2] = z[4];

        // clamp/scale setpoint vel
        let setpoint = clamp_scale_vector(setpoint, &BODY_VEL_LIM);
        self.debug_telemetry.commanded_body_velocity.copy_from_slice(setpoint.as_slice());

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