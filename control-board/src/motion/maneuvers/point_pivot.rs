use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::build_pivot_params;
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedPointPivotTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::pivot_trajectory::PivotTrajectory;
use ateam_controls::ControlsError;

pub struct PointPivotManeuver;

impl PointPivotManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for PointPivotManeuver {
    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::PointPivot(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
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
            ManeuverExtendedTelemetry::PointPivot(ExtendedPointPivotTelemetry { cmd_echo: c });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
