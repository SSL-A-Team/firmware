use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::build_linear_params;
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::{BodyControlCommand, BodyControlManeuverExtendedTelemetry, ExtendedPointLineTelemetry};
use ateam_controls::linear_trajectory::LinearTrajectory;
use ateam_controls::{ControlsError, Vector2f};

pub struct PointLineManeuver;

impl PointLineManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for PointLineManeuver {
    fn entry(&mut self, _cmd: BodyControlCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: BodyControlCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, BodyControlManeuverExtendedTelemetry), ControlsError> {
        let BodyControlCommand::PointLine(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        };

        // Vision required: disable wheels and return zero setpoints until a
        // fresh vision sample arrives.
        if !ctx.vision_active() {
            ctx.wheels_disabled = true;
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        }

        let params = build_linear_params(
            c.max_vel_colinear,
            c.max_vel_perp,
            c.max_vel_angular,
            c.max_accel_colinear,
            c.max_accel_perp,
            c.max_accel_angular,
            c.colinear_start_thresh,
        );
        let start_point = Vector2f::new(c.start_x, c.start_y);
        let line_dir = Vector2f::new(c.dir_x, c.dir_y);

        let setpoints = ctx.run_traj_track(cmd, |seed| {
            let traj = LinearTrajectory::from_point(
                seed,
                c.target_x,
                c.target_y,
                start_point,
                line_dir,
                c.line_velocity,
                params,
            )?;
            Ok(TrackedTrajectory::Linear(traj))
        })?;

        let telem =
            BodyControlManeuverExtendedTelemetry::PointLine(ExtendedPointLineTelemetry { cmd_echo: c });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
