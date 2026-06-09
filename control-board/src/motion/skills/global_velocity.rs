use crate::motion::control_context::{CommandFrame, ControlContext, SkillSetpoints};
use crate::motion::skills::MotionSkill;
use ateam_common_packets::bindings::{ExtendedGlobalVelocityTelemetry, GlobalVelocityCommand};
use ateam_common_packets::radio::SkillExtendedTelemetry;
use ateam_controls::bangbang_trajectory::TrajectoryParams;
use ateam_controls::ControlsError;

pub struct GlobalVelocitySkill;

impl GlobalVelocitySkill {
    pub fn new() -> Self {
        Self
    }
}

impl MotionSkill for GlobalVelocitySkill {
    type Command = GlobalVelocityCommand;

    fn entry(&mut self, _cmd: GlobalVelocityCommand, _ctx: &mut ControlContext) {
        
    }

    fn update(
        &mut self,
        cmd: GlobalVelocityCommand,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError> {
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

        let telem = SkillExtendedTelemetry::GlobalVelocity(ExtendedGlobalVelocityTelemetry {
            cmd_echo: cmd,
        });

        Ok((SkillSetpoints { body_twist, body_accel }, telem))
    }

    fn reset(&mut self) {

    }
}
