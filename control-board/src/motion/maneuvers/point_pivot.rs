use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::build_pivot_params;
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::{BodyControlCommand, BodyControlManeuverExtendedTelemetry, ExtendedPointPivotTelemetry};
use ateam_controls::pivot_trajectory::PivotTrajectory;
use ateam_controls::ControlsError;

pub struct PointPivotManeuver;

impl PointPivotManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for PointPivotManeuver {
    fn entry(&mut self, _cmd: BodyControlCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: BodyControlCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, BodyControlManeuverExtendedTelemetry), ControlsError> {
        let BodyControlCommand::PointPivot(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        };

        let traj_params = build_pivot_params(
            c.max_angular_vel,
            c.max_angular_acc,
            c.orbit_radius,
            c.inset_angle,
            c.direction,
            c.compute_inset_angle,
        );
        let setpoints = ctx.run_traj_track(cmd, |seed| {
            let traj =
                PivotTrajectory::from_target_point(seed, c.target_x, c.target_y, traj_params)?;
            Ok(TrackedTrajectory::Pivot(traj))
        })?;

        let telem =
            BodyControlManeuverExtendedTelemetry::PointPivot(ExtendedPointPivotTelemetry { cmd_echo: c });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
