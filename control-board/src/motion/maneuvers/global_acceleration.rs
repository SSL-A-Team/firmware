use crate::motion::control_context::{ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedGlobalAccelerationTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::{ControlsError, Vector3f};

pub struct GlobalAccelerationManeuver;

impl GlobalAccelerationManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for GlobalAccelerationManeuver {
    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::GlobalAcceleration(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
        };

        let target_accel = c.as_vec3f();
        let state_estimate = ctx.state_estimate;
        let next_state = ctx.robot_model.a * state_estimate + ctx.robot_model.b * target_accel;
        let body_twist: Vector3f = next_state.fixed_rows::<3>(3).into();

        let telem =
            ManeuverExtendedTelemetry::GlobalAcceleration(ExtendedGlobalAccelerationTelemetry {
                cmd_echo: c,
            });

        Ok((
            ManeuverSetpoints {
                body_twist,
                body_accel: target_accel,
            },
            telem,
        ))
    }

    fn reset(&mut self) {}
}
