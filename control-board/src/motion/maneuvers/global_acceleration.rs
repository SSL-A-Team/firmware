use crate::motion::control_context::{ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::{BodyControlCommand, BodyControlManeuverExtendedTelemetry, ExtendedGlobalAccelerationTelemetry};
use ateam_controls::{ControlsError, Vector3f};

pub struct GlobalAccelerationManeuver;

impl GlobalAccelerationManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for GlobalAccelerationManeuver {
    fn entry(&mut self, _cmd: BodyControlCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: BodyControlCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, BodyControlManeuverExtendedTelemetry), ControlsError> {
        let BodyControlCommand::GlobalAcceleration(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        };

        let target_accel = c.as_vec3f();
        let state_estimate = ctx.state_estimate;
        let next_state = ctx.robot_model.a * state_estimate + ctx.robot_model.b * target_accel;
        let body_twist: Vector3f = next_state.fixed_rows::<3>(3).into();

        let telem =
            BodyControlManeuverExtendedTelemetry::GlobalAcceleration(ExtendedGlobalAccelerationTelemetry {
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
