use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::{BodyControlCommand, BodyControlManeuverExtendedTelemetry, ExtendedGlobalPositionTelemetry};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::ControlsError;

pub struct GlobalPositionManeuver;

impl GlobalPositionManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for GlobalPositionManeuver {
    fn entry(&mut self, _cmd: BodyControlCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: BodyControlCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, BodyControlManeuverExtendedTelemetry), ControlsError> {
        let BodyControlCommand::GlobalPosition(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        };

        // Vision required: disable wheels and return zero setpoints until a
        // fresh vision sample arrives.
        if !ctx.vision_active() {
            ctx.wheels_disabled = true;
            return Ok((ManeuverSetpoints::zero(), BodyControlManeuverExtendedTelemetry::Off));
        }

        let default_params = TrajectoryParams::default();
        let traj_params = TrajectoryParams {
            max_vel_linear: if c.max_linear_vel != 0.0 {
                c.max_linear_vel
            } else {
                default_params.max_vel_linear
            },
            max_vel_angular: if c.max_angular_vel != 0.0 {
                c.max_angular_vel
            } else {
                default_params.max_vel_angular
            },
            max_accel_linear: if c.max_linear_acc != 0.0 {
                c.max_linear_acc
            } else {
                default_params.max_accel_linear
            },
            max_accel_angular: if c.max_angular_acc != 0.0 {
                c.max_angular_acc
            } else {
                default_params.max_accel_angular
            },
        };
        let target_pose = c.as_vec3f();

        let setpoints = ctx.run_traj_track(cmd, |seed| {
            let traj = BangBangTraj3D::from_target_pose(seed, target_pose, traj_params)?;
            Ok(TrackedTrajectory::BangBang(traj))
        })?;

        let telem = BodyControlManeuverExtendedTelemetry::GlobalPosition(ExtendedGlobalPositionTelemetry {
            cmd_echo: c,
        });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
