use ateam_controls::trajectory::Trajectory;
use ateam_controls::{wrap_angle, z_rotation_mat, Vector3f, Vector6f};

/// Frame a commanded acceleration is expressed in.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AccelFrame {
    Global,
    /// Robot frame; rotates with the trajectory's own heading, not the estimate.
    Local,
    /// Robot frame; rotates with the state-estimate heading supplied each tick
    /// via `set_body_heading` (falls back to the trajectory heading until set).
    Body,
}

/// Trajectory that integrates a constant commanded acceleration from a seed state.
pub struct AccelTrajectory {
    state: Vector6f,
    accel: Vector3f,
    frame: AccelFrame,
    body_heading: Option<f32>,
}

impl AccelTrajectory {
    pub fn new(seed: Vector6f, accel: Vector3f, frame: AccelFrame) -> Self {
        Self {
            state: seed,
            accel,
            frame,
            body_heading: None,
        }
    }

    /// Heading used to rotate `AccelFrame::Body` accelerations into the global frame.
    pub fn set_body_heading(&mut self, theta: f32) {
        self.body_heading = Some(theta);
    }

    fn global_accel(&self) -> Vector3f {
        match self.frame {
            AccelFrame::Global => self.accel,
            AccelFrame::Local => z_rotation_mat(self.state.z) * self.accel,
            AccelFrame::Body => {
                z_rotation_mat(self.body_heading.unwrap_or(self.state.z)) * self.accel
            }
        }
    }
}

impl Trajectory for AccelTrajectory {
    fn tick(&mut self, dt: f32) {
        let accel = self.global_accel();
        let vel: Vector3f = self.state.fixed_rows::<3>(3).into();
        let pos: Vector3f = self.state.fixed_rows::<3>(0).into();
        let mut next_pos = pos + vel * dt + accel * (0.5 * dt * dt);
        next_pos.z = wrap_angle(next_pos.z);
        self.state.fixed_rows_mut::<3>(0).copy_from(&next_pos);
        self.state.fixed_rows_mut::<3>(3).copy_from(&(vel + accel * dt));
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        (self.state, self.global_accel())
    }
}
