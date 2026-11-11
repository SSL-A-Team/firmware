use crate::motion::control_context::{
    ControlContext, ManeuverSetpoints, TrackedTrajectory, FEASIBLE_ACCEL_MARGIN,
    MIN_PLANNED_ACCEL_LINEAR,
};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedGlobalPositionTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::ControlsError;

pub struct GlobalPositionManeuver;

impl GlobalPositionManeuver {
    pub fn new() -> Self {
        Self
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
        let target_pose = c.as_vec3f();

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

        let setpoints = ctx.run_traj_track(cmd, |seed| {
            let traj = BangBangTraj3D::from_target_pose(seed, target_pose, traj_params)?;
            Ok(TrackedTrajectory::BangBang(traj))
        })?;

        let telem = ManeuverExtendedTelemetry::GlobalPosition(ExtendedGlobalPositionTelemetry {
            cmd_echo: c,
        });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
