use crate::motion::control_context::{ControlContext, ManeuverSetpoints};
use crate::motion::velocity_ramp::VelocityFrame;
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedLocalVelocityTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::bangbang_trajectory::TrajectoryParams;
use ateam_controls::ControlsError;

pub struct LocalVelocityManeuver;

impl LocalVelocityManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for LocalVelocityManeuver {
    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::LocalVelocity(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
        };

        // TIGERs-style velocity ramp: velocity and acceleration feed-forward only,
        // no position feedback (see ControlContext::run_vel_track).
        let p = TrajectoryParams::default();
        let max_accel_linear = if c.max_linear_acc != 0.0 {
            c.max_linear_acc
        } else {
            p.max_accel_linear
        };
        let max_accel_angular = if c.max_angular_acc != 0.0 {
            c.max_angular_acc
        } else {
            p.max_accel_angular
        };
        let setpoints = ctx.run_vel_track(
            cmd,
            VelocityFrame::Local,
            c.as_vec3f(),
            p.max_vel_linear,
            p.max_vel_angular,
            max_accel_linear,
            max_accel_angular,
        )?;

        let telem = ManeuverExtendedTelemetry::LocalVelocity(ExtendedLocalVelocityTelemetry {
            cmd_echo: c,
        });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
