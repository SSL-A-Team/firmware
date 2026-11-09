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
    fn entry(&mut self, _cmd: ManeuverCommand, ctx: &mut ControlContext) {
        // Accel maneuvers don't track a trajectory, so drop the previous one rather
        // than let it go stale; the next tracking maneuver seeds from the estimate.
        ctx.reset_trajectory();
    }

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
        let est_twist: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let body_twist = est_twist + target_accel * ctx.dt;

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
