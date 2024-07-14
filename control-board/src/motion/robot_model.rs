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
    t_body_to_wheel: Matrix4x3<f32>,
    t_wheel_to_body: Matrix3x4<f32>,
}

impl RobotModel {
    fn t_body_to_wheel_from_model(robot_constants: RobotConstants) -> Matrix4x3<f32> {
        let theta = &robot_constants.wheel_angles_rad;
        let wheel_dist = &robot_constants.wheel_dist_to_cent_m;

        let neg_r = -(&robot_constants.wheel_radius_m);

        Matrix4x3::new(
            cosf(theta[0]) / neg_r[0], sinf(theta[0]) / neg_r[0], -wheel_dist[0] / neg_r[0],
            cosf(theta[1]) / neg_r[1], sinf(theta[1]) / neg_r[1], -wheel_dist[1] / neg_r[1],
            cosf(theta[2]) / neg_r[2], sinf(theta[2]) / neg_r[2], -wheel_dist[2] / neg_r[2],
            cosf(theta[3]) / neg_r[3], sinf(theta[3]) / neg_r[3], -wheel_dist[3] / neg_r[3],
        )
    }

    pub fn new(robot_constants: RobotConstants) -> RobotModel {
        // Construct the backward (body velocity to wheel angular velocity) and forward (wheel to body) dynamics.
        let t_body_to_wheel = Self::t_body_to_wheel_from_model(robot_constants);
        let ta = t_body_to_wheel.transpose() * t_body_to_wheel;
        if !ta.is_invertible() {
            defmt::error!("Body velocity to wheel TA matrix is not invertible, cannot calculate the forward dynamics!")
        } 
        let t_wheel_to_body = ta.try_inverse().unwrap() * t_body_to_wheel.transpose();

        RobotModel {
            t_body_to_wheel: t_body_to_wheel,
            t_wheel_to_body: t_wheel_to_body,
        }
    }

    pub fn robot_vel_to_wheel_vel(&self, robot_vel: &Vector3<f32>) -> Vector4<f32> {
        self.t_body_to_wheel * robot_vel
    }

    pub fn wheel_vel_to_robot_vel(&self, wheel_vels: &Vector4<f32>) -> Vector3<f32> {
        self.t_wheel_to_body * wheel_vels
    }
}