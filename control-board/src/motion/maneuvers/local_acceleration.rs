use crate::motion::control_context::{ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::{BodyControlCommand, BodyControlManeuverExtendedTelemetry, ExtendedLocalAccelerationTelemetry};
use ateam_controls::{z_rotation_mat, ControlsError, Vector3f};

pub struct LocalAccelerationManeuver;

impl LocalAccelerationManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for LocalAccelerationManeuver {
    fn entry(&mut self, _cmd: BodyControlCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: BodyControlCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, BodyControlManeuverExtendedTelemetry), ControlsError> {
        let BodyControlCommand::LocalAcceleration(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        };

        let state_estimate = ctx.state_estimate;
        let target_accel = z_rotation_mat(state_estimate.z) * c.as_vec3f();
        let next_state = ctx.robot_model.a * state_estimate + ctx.robot_model.b * target_accel;
        let body_twist: Vector3f = next_state.fixed_rows::<3>(3).into();

        let telem =
            BodyControlManeuverExtendedTelemetry::LocalAcceleration(ExtendedLocalAccelerationTelemetry {
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
