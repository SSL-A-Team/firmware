use alloc::vec::Vec;
use nalgebra::Vector3;

use super::constant_gain_kalman_filter::CgKalmanFilter;
use super::pid::PidController;
use super::robot_model::RobotModel;

use super::params::body_vel_filter_params::{
    F, B, H, Q, R, P, K,
    KfNumStates, KfNumControlInputs, KfNumObservations,
};

pub struct BodyVelocityController {
    robot_model: RobotModel,
    body_vel_filter: CgKalmanFilter<3, 4, 5>,
    body_vel_controller: PidController<3>,
    body_velocity_limit: Vector3<f32>,
    body_acceleration_limit: Vector3<f32>,
    wheel_acceleration_limits: Vector4<f32>,
    cmd_wheel_velocities: Vector4<f32>,
}

impl BodyVelocityController {
    pub fn new_from_global_params() -> BodyVelocityController {
        let body_vel_filter: CgKalmanFilter<KfNumStates, KfNumControlInputs, KfNumObservations> = 
                CgKalmanFilter::new(&F, &B, &H, &Q, &R, &P, &K);

        let body_vel_controller: PidController<KfNumStates> =
                PidController::from_gains_matrix();
    }

    pub fn new(robot_model: RobotModel,
            body_vel_filter: CgKalmanFilter<3, 4, 5>,
            pid_controller: PidController<3>,
            bv_limit: Vector3<f32>,
            ba_limit: Vector3<f32>,
            wa_limit: Vector4<f32>
    ) -> BodyVelocityController {
        BodyVelocityController {
            robot_model: robot_model,
            body_vel_filter: body_vel_filter,
            body_vel_controller: pid_controller,
            body_velocity_limit: bv_limit,
            body_acceleration_limit: ba_limit,
            wheel_acceleration_limits: wa_limit,
            cmd_wheel_velocities: Vector4::zeros(),
        }
    }

    fn control_update(&self, setpoint: Vector3<f32>, wheel_velocities: Vector4<f32>, gryo_theta: f32) {
        // TODO clamp/scale setpoint vel

        // construct the observation vector the KF expects
        let z: Vector5<f32> = Vector5::new(wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3], gryo_theta);

        // get latest wheel vals and gyro (observe the state)
        // process observation input into CGKF (call update())
        self.body_vel_filter.update(z);

        // read the state estimate
        let y = self.body_vel_filter.get_state();

        // apply control policy
        // TODO make this a global or compute dynamically
        self.body_vel_controller.calculate(setpoints, y, 1.0 / 120.0);
        let u = self.body_vel_controller.get_u();

        // TODO clamp/scale u vel
        // TODO clamp/scale u accel

        // process body vel into the wheel velocities
        let wheel_u = self.robot_model.robot_vel_to_wheel_vel(u);

        // apply control law against control input from soccer + current estimate of the state
        // use control law adjusted value to predict the next cycle's state
        // e.g. call CGKF.predict(post PID u)
        self.body_vel_filter.predict(&wheel_u);

        // TODO clamp/scale omegas

        // save command state
        self.cmd_wheel_velocities = wheel_u;
    }

    fn get_wheel_velocities(&self) -> Vector4<f32> {
        self.cmd_wheel_velocities
    }
}