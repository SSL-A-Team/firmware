use nalgebra::{Vector3, Vector4, Vector6, Matrix3, Matrix4x3};
use libm::{sinf, cosf};

#[derive(Clone, Copy)]
pub struct RobotConstants {
    // model params
    pub wheel_angles_rad: Vector4<f32>,
    pub wheel_radius_m: Vector4<f32>,
    pub wheel_dist_to_cent_m: Vector4<f32>,
}

pub struct RobotModel {
    // model params
    robot_constants: RobotConstants,

    // state
    robot_state_estimate: Vector6<f32>,

    // transformation matrices
    t_robot: Matrix3<f32>,
    t_wheel: Matrix4x3<f32>,


    robot_desired_state: Vector6<f32>,

    cmd_wheel_duties: Vector4<f32>,
}

impl RobotModel {
    fn T_wheel_from_model(robot_constants: RobotConstants) -> Matrix4x3<f32> {
        // theta is the mounted wheel angle
        // beta is the angle to the mount location
        // if the mounted angle is always the same as the radial from the center
        // theta and beta always cancel in the angular velocity components and beta is unnecessary
        let theta = &robot_constants.wheel_angles_rad;
        let beta = Vector4::new(theta[0], theta[1], theta[2], theta[3]);
        let l = &robot_constants.wheel_dist_to_cent_m;
        let r = &robot_constants.wheel_radius_m;

        Matrix4x3::new(
            cosf(theta[0]) / r[0], sinf(theta[0]) / r[0], (l[0] * cosf(theta[0] - beta[0])) / r[0],
            cosf(theta[1]) / r[1], sinf(theta[1]) / r[1], (l[1] * cosf(theta[1] - beta[1])) / r[1],
            cosf(theta[2]) / r[2], sinf(theta[2]) / r[2], (l[2] * cosf(theta[2] - beta[2])) / r[2],
            cosf(theta[3]) / r[3], sinf(theta[3]) / r[3], (l[3] * cosf(theta[3] - beta[3])) / r[3],
        )
    }

    pub fn new(robot_constants: RobotConstants) -> RobotModel {
        RobotModel {
            robot_constants,
            robot_state_estimate: Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            t_robot: Matrix3::new_rotation(0.0),
            t_wheel: Self::T_wheel_from_model(robot_constants),
            robot_desired_state: Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            cmd_wheel_duties: Vector4::new(0.0, 0.0, 0.0, 0.0),
        }
    }

    pub fn robot_vel_to_wheel_vel(&self, robot_vel: Vector3<f32>) -> Vector4<f32> {
        self.t_wheel * robot_vel
    }
}