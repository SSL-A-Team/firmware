use crate::motion::accel_trajectory::{AccelFrame, AccelTrajectory};
use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedLocalAccelerationTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::ControlsError;

/// Integrates the commanded robot-frame acceleration into the trajectory and tracks it
/// closed loop (pose PID + feedforward), like the velocity and position maneuvers.
pub struct LocalAccelerationManeuver;

impl LocalAccelerationManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for LocalAccelerationManeuver {
    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::LocalAcceleration(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
        };

        let accel = c.as_vec3f();
        let setpoints = ctx.run_traj_track(cmd, |seed| {
            Ok(TrackedTrajectory::Accel(AccelTrajectory::new(
                seed,
                accel,
                AccelFrame::Local,
            )))
        })?;

        let telem = ManeuverExtendedTelemetry::LocalAcceleration(ExtendedLocalAccelerationTelemetry { cmd_echo: c });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
