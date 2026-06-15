use crate::motion::control_context::{CommandFrame, ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::{
    ExtendedLocalAccelerationTelemetry, LocalAccelerationCommand,
};
use ateam_common_packets::radio::ManeuverExtendedTelemetry;
use ateam_controls::ControlsError;

pub struct LocalAccelerationManeuver;

impl LocalAccelerationManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for LocalAccelerationManeuver {
    type Command = LocalAccelerationCommand;

    fn entry(&mut self, _cmd: LocalAccelerationCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: LocalAccelerationCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let (body_twist, body_accel) =
            ctx.accel_control_policy(cmd.as_vec3f(), CommandFrame::Local);

        let telem = ManeuverExtendedTelemetry::LocalAcceleration(ExtendedLocalAccelerationTelemetry {
            cmd_echo: cmd,
        });

        Ok((
            ManeuverSetpoints {
                body_twist,
                body_accel,
            },
            telem,
        ))
    }

    fn reset(&mut self) {}
}
