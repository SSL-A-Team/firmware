use crate::motion::control_context::{CommandFrame, ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::{
    ExtendedGlobalAccelerationTelemetry, GlobalAccelerationCommand,
};
use ateam_common_packets::radio::ManeuverExtendedTelemetry;
use ateam_controls::ControlsError;

pub struct GlobalAccelerationManeuver;

impl GlobalAccelerationManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for GlobalAccelerationManeuver {
    type Command = GlobalAccelerationCommand;

    fn entry(&mut self, _cmd: GlobalAccelerationCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: GlobalAccelerationCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let (body_twist, body_accel) =
            ctx.accel_control_policy(cmd.as_vec3f(), CommandFrame::Global);

        let telem =
            ManeuverExtendedTelemetry::GlobalAcceleration(ExtendedGlobalAccelerationTelemetry {
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
