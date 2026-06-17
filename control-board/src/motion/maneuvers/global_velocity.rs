use crate::motion::control_context::{CommandFrame, ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::{ExtendedGlobalVelocityTelemetry, GlobalVelocityCommand};
use ateam_common_packets::radio::ManeuverExtendedTelemetry;
use ateam_controls::bangbang_trajectory::TrajectoryParams;
use ateam_controls::ControlsError;

pub struct GlobalVelocityManeuver;

impl GlobalVelocityManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for GlobalVelocityManeuver {
    type Command = GlobalVelocityCommand;

    fn entry(&mut self, _cmd: GlobalVelocityCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: GlobalVelocityCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let default_params = TrajectoryParams::default();

        let traj_params = TrajectoryParams {
            max_vel_linear: default_params.max_vel_linear,
            max_vel_angular: default_params.max_vel_angular,
            max_accel_linear: if cmd.max_linear_acc != 0.0 {
                cmd.max_linear_acc
            } else {
                default_params.max_accel_linear
            },
            max_accel_angular: if cmd.max_angular_acc != 0.0 {
                cmd.max_angular_acc
            } else {
                default_params.max_accel_angular
            },
        };

        let (body_twist, body_accel) =
            ctx.twist_control_policy(cmd.as_vec3f(), CommandFrame::Global, traj_params)?;

        let telem = ManeuverExtendedTelemetry::GlobalVelocity(ExtendedGlobalVelocityTelemetry {
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
