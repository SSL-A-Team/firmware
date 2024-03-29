use nalgebra::{Matrix3x4, Matrix4x3, Vector3, Vector4};
use libm::{sinf, cosf};

#[derive(Clone, Copy)]
pub struct RobotConstants {
    // model params
    pub wheel_angles_rad: Vector4<f32>,
    pub wheel_radius_m: Vector4<f32>,
    pub wheel_dist_to_cent_m: Vector4<f32>,
}

#[derive(Clone, Copy)]
pub struct RobotModel {
    t_robot_to_wheel: Matrix4x3<f32>,
    t_wheel_to_robot: Matrix3x4<f32>,
}

impl RobotModel {
    fn T_body_wheel_from_model(robot_constants: RobotConstants) -> Matrix4x3<f32> {
        let theta = &robot_constants.wheel_angles_rad;
        let l = &robot_constants.wheel_dist_to_cent_m;
        let r = &robot_constants.wheel_radius_m;

        Matrix4x3::new(
            cosf(theta[0]) / r[0], sinf(theta[0]) / r[0], l[0] / r[0],
            cosf(theta[1]) / r[1], sinf(theta[1]) / r[1], l[1] / r[1],
            cosf(theta[2]) / r[2], sinf(theta[2]) / r[2], l[2] / r[2],
            cosf(theta[3]) / r[3], sinf(theta[3]) / r[3], l[3] / r[3],
        )
    }

    pub fn new(robot_constants: RobotConstants) -> RobotModel {
        // construct the backward (bot to wheel) and forward (wheel to bot) dynamics
        let t_robot_to_wheel = Self::T_body_wheel_from_model(robot_constants);
        let ta = t_robot_to_wheel.transpose() * t_robot_to_wheel;
        if !ta.is_invertible() {
            defmt::error!("robot to wheel TA matrix is not invertible, cannot calculate the forward dynamics")
        } 
        let t_wheel_to_robot = ta.try_inverse().unwrap() * t_robot_to_wheel.transpose();

        RobotModel {
            t_robot_to_wheel: t_robot_to_wheel,
            t_wheel_to_robot: t_wheel_to_robot,
        }
    }

    pub fn robot_vel_to_wheel_vel(&self, robot_vel: Vector3<f32>) -> Vector4<f32> {
        self.t_robot_to_wheel * robot_vel
    }

    pub fn wheel_vel_to_robot_vel(&self, wheel_vels: Vector4<f32>) -> Vector3<f32> {
        self.t_wheel_to_robot * wheel_vels
    }
}