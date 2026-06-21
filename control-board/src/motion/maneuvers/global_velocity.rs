use crate::motion::control_context::{ControlContext, ManeuverSetpoints, TrackedTrajectory};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::ExtendedGlobalVelocityTelemetry;
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use ateam_controls::ControlsError;

pub struct GlobalVelocityManeuver;

impl GlobalVelocityManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for GlobalVelocityManeuver {
    fn entry(&mut self, _cmd: ManeuverCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let ManeuverCommand::GlobalVelocity(c) = cmd else {
            return Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off));
        };

        let default_params = TrajectoryParams::default();
        let traj_params = TrajectoryParams {
            max_vel_linear: default_params.max_vel_linear,
            max_vel_angular: default_params.max_vel_angular,
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
        let target_twist = c.as_vec3f();

        let setpoints = ctx.run_traj_track(cmd, |seed| {
            let traj = BangBangTraj3D::from_target_twist(seed, target_twist, traj_params)?;
            Ok(TrackedTrajectory::BangBang(traj))
        })?;

        let telem = ManeuverExtendedTelemetry::GlobalVelocity(ExtendedGlobalVelocityTelemetry {
            cmd_echo: c,
        });
        Ok((setpoints, telem))
    }

    fn reset(&mut self) {}
}
