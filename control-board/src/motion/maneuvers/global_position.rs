use crate::motion::control_context::{
    ControlContext, ManeuverSetpoints, TrackedTrajectory, FEASIBLE_ACCEL_MARGIN,
    MIN_PLANNED_ACCEL_LINEAR,
};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedGlobalPositionTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::ControlsError;
use core::f32::consts::PI;
use libm::{fabsf, remainderf};

/// Orientation-target PT1 after TIGERs Mannheim `TrajBangBangUpdate`
/// (traj_bang_bang.c): target changes below `JUMP_RAD` are low-passed with time
/// constant `T_S`, larger ones pass through. Without it, every small change of a
/// streamed heading target replans a fresh angular bang-bang at full acceleration.
struct OrientTargetLag {
    out: f32,
    last_in: f32,
    last_target: f32,
}

impl OrientTargetLag {
    const T_S: f32 = 0.1;
    const JUMP_RAD: f32 = 0.05;
    /// Settled within this the output snaps to the target, so replanning stops.
    const SNAP_RAD: f32 = 1e-4;

    fn new(target: f32) -> Self {
        Self { out: target, last_in: target, last_target: target }
    }

    /// Filter `target` (any angle); the output is continuous (multi-turn).
    fn process(&mut self, target: f32, dt: f32) -> f32 {
        let target = self.last_target + remainderf(target - self.last_target, 2.0 * PI);
        if fabsf(target - self.last_target) > Self::JUMP_RAD {
            self.out = target;
            self.last_in = target;
        } else {
            // Tustin PT1 (TIGERs LagElementPT1, K = 1).
            let a = (2.0 * Self::T_S - dt) / (2.0 * Self::T_S + dt);
            let b = dt / (2.0 * Self::T_S + dt);
            self.out = b * (target + self.last_in) + a * self.out;
            self.last_in = target;
            if fabsf(target - self.out) < Self::SNAP_RAD {
                self.out = target;
            }
        }
        self.last_target = target;
        self.out
    }
}

pub struct GlobalPositionManeuver {
    orient_lag: Option<OrientTargetLag>,
}

impl GlobalPositionManeuver {
    pub fn new() -> Self {
        Self { orient_lag: None }
    }
}

impl MotionManeuver for GlobalPositionManeuver {
    const REQUIRES_VISION: bool = true;

    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::GlobalPosition(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
        };

        let default_params = TrajectoryParams::default();
        let traj_params = TrajectoryParams {
            max_vel_linear: if c.max_linear_vel != 0.0 {
                c.max_linear_vel
            } else {
                default_params.max_vel_linear
            },
            max_vel_angular: if c.max_angular_vel != 0.0 {
                c.max_angular_vel
            } else {
                default_params.max_vel_angular
            },
            max_accel_linear: if c.max_linear_acc != 0.0 {
                c.max_linear_acc
            } else {
                default_params.max_accel_linear
            },
            max_accel_angular: if c.max_angular_acc != 0.0 {
                c.max_angular_acc
            } else {
                default_params.max_accel_angular
            },
        };
        let dt = ctx.dt;
        let theta = match self.orient_lag.as_mut() {
            Some(lag) => lag.process(c.global_theta, dt),
            None => {
                self.orient_lag = Some(OrientTargetLag::new(c.global_theta));
                c.global_theta
            }
        };
        let mut c_filt = c;
        c_filt.global_theta = remainderf(theta, 2.0 * PI);
        let target_pose = c_filt.as_vec3f();

        // Capability-aware acceleration limit: never plan more linear acceleration
        // than the motors can deliver along the travel direction (in the robot frame,
        // over the headings it passes through), leaving headroom for feedback. Keeps
        // the trajectory trackable instead of running away and being chased by the
        // position feedback (overspeed), while forward moves keep full capability.
        let mut traj_params = traj_params;
        let est = ctx.state_estimate;
        let delta_xy =
            nalgebra::SVector::<f32, 2>::new(target_pose.x - est[0], target_pose.y - est[1]);
        if delta_xy.norm() > 0.005 {
            let feasible = ctx.feasible_linear_accel_for_move(delta_xy, est[2], target_pose.z);
            let limit = (FEASIBLE_ACCEL_MARGIN * feasible).max(MIN_PLANNED_ACCEL_LINEAR);
            traj_params.max_accel_linear = traj_params.max_accel_linear.min(limit);
        }

        // Replan whenever the filtered target moves, not just when the raw one does.
        let cmd_filt = ManeuverCommand::GlobalPosition(c_filt);
        let setpoints = ctx.run_traj_track(cmd_filt, |seed| {
            let traj = BangBangTraj3D::from_target_pose(seed, target_pose, traj_params)?;
            Ok(TrackedTrajectory::BangBang(traj))
        })?;

        let telem = ManeuverExtendedTelemetry::GlobalPosition(ExtendedGlobalPositionTelemetry {
            cmd_echo: c,
        });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {
        self.orient_lag = None;
    }
}
