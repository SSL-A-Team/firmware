//! First-order velocity trajectory for the velocity maneuvers, after TIGERs Mannheim
//! `TrajFirstOrder2D/1D` and `TrajGeneratorCreateLocalVelXY/W` (Firmware
//! src/shared/math/traj_1order.c, traj_generator.c): the velocity ramps toward the
//! target at the maximum acceleration (XY as one vector, so direction is preserved,
//! W separately). The acceleration reported is the average over the next control
//! step, so it fades to the remaining velocity difference instead of switching
//! between +/- the maximum when the target is reached (TIGERs' numeric-difference
//! orientation acceleration).

use ateam_controls::trajectory::Trajectory;
use ateam_controls::{wrap_angle, z_rotation_mat, Vector3f, Vector6f};
use libm::{fabsf, hypotf};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum VelocityFrame {
    Global,
    /// Robot frame; rotated to global with the state-estimate heading supplied
    /// each tick via `set_body_heading`.
    Local,
}

pub struct VelocityRamp {
    frame: VelocityFrame,
    /// Integrated global pose (telemetry only, no feedback is applied on it).
    pos: Vector3f,
    /// Current velocity in `frame`.
    vel: Vector3f,
    target: Vector3f,
    max_accel_linear: f32,
    max_accel_angular: f32,
    body_heading: f32,
    dt: f32,
}

impl VelocityRamp {
    /// Start from `seed` (global pose and velocity) with heading `body_heading`.
    pub fn new(
        seed: Vector6f,
        body_heading: f32,
        frame: VelocityFrame,
        target: Vector3f,
        max_accel_linear: f32,
        max_accel_angular: f32,
        dt: f32,
    ) -> Self {
        let vel_global: Vector3f = seed.fixed_rows::<3>(3).into();
        let vel = match frame {
            VelocityFrame::Global => vel_global,
            VelocityFrame::Local => z_rotation_mat(-body_heading) * vel_global,
        };
        Self {
            frame,
            pos: seed.fixed_rows::<3>(0).into(),
            vel,
            target,
            max_accel_linear,
            max_accel_angular,
            body_heading,
            dt,
        }
    }

    pub fn set_body_heading(&mut self, theta: f32) {
        self.body_heading = theta;
    }

    /// Velocity (in `frame`) one control step ahead.
    fn next_vel(&self) -> Vector3f {
        let (dx, dy) = (self.target.x - self.vel.x, self.target.y - self.vel.y);
        let d_xy = hypotf(dx, dy);
        let step_xy = self.max_accel_linear * self.dt;
        let (nx, ny) = if d_xy <= step_xy {
            (self.target.x, self.target.y)
        } else {
            (self.vel.x + dx * step_xy / d_xy, self.vel.y + dy * step_xy / d_xy)
        };
        let dw = self.target.z - self.vel.z;
        let step_w = self.max_accel_angular * self.dt;
        let nw = if fabsf(dw) <= step_w {
            self.target.z
        } else {
            self.vel.z + step_w * dw.signum()
        };
        Vector3f::new(nx, ny, nw)
    }

    fn to_global(&self, v: Vector3f) -> Vector3f {
        match self.frame {
            VelocityFrame::Global => v,
            VelocityFrame::Local => z_rotation_mat(self.body_heading) * v,
        }
    }
}

impl Trajectory for VelocityRamp {
    fn tick(&mut self, dt: f32) {
        let next = self.next_vel();
        let mean_vel_global = self.to_global((self.vel + next) * 0.5);
        self.pos += mean_vel_global * dt;
        self.pos.z = wrap_angle(self.pos.z);
        self.vel = next;
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        let accel = self.to_global((self.next_vel() - self.vel) / self.dt);
        let vel = self.to_global(self.vel);
        let mut state = Vector6f::zeros();
        state.fixed_rows_mut::<3>(0).copy_from(&self.pos);
        state.fixed_rows_mut::<3>(3).copy_from(&vel);
        (state, accel)
    }
}
