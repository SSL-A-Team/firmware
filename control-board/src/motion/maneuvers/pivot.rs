use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedPivotTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::pivot_trajectory::{PivotDirection, PivotParams, PivotTrajectory};
use ateam_controls::ControlsError;

pub struct PivotManeuver;

impl PivotManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for PivotManeuver {
    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::Pivot(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
        };

        let default_params = PivotParams::default();
        let traj_params = PivotParams {
            max_vel_angular: if c.max_angular_vel != 0.0 {
                c.max_angular_vel
            } else {
                default_params.max_vel_angular
            },
            max_accel_angular: if c.max_angular_acc != 0.0 {
                c.max_angular_acc
            } else {
                default_params.max_accel_angular
            },
            orbit_radius: if c.orbit_radius != 0.0 {
                c.orbit_radius
            } else {
                default_params.orbit_radius
            },
            inset_angle: c.inset_angle,
            compute_inset_angle: false,
            direction: PivotDirection::Forward,
        };
        let setpoints = ctx.run_traj_track(cmd, |seed| {
            let traj = PivotTrajectory::from_target_heading(seed, c.global_theta, traj_params)?;
            Ok(TrackedTrajectory::Pivot(traj))
        })?;

        let telem = ManeuverExtendedTelemetry::Pivot(ExtendedPivotTelemetry { cmd_echo: c });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
