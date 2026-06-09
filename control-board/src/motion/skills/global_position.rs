use crate::motion::control_context::{ControlContext, SkillSetpoints};
use crate::motion::skills::MotionSkill;
use ateam_common_packets::bindings::{ExtendedGlobalPositionTelemetry, GlobalPositionCommand};
use ateam_common_packets::radio::SkillExtendedTelemetry;
use ateam_controls::bangbang_trajectory::TrajectoryParams;
use ateam_controls::ControlsError;

pub struct GlobalPositionSkill;

impl GlobalPositionSkill {
    pub fn new() -> Self {
        Self
    }
}

impl MotionSkill for GlobalPositionSkill {
    type Command = GlobalPositionCommand;

    fn entry(&mut self, _cmd: GlobalPositionCommand, _ctx: &mut ControlContext) {

    }

    fn update(
        &mut self,
        cmd: GlobalPositionCommand,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError> {
        let default_params = TrajectoryParams::default();

        let traj_params = TrajectoryParams {
            max_vel_linear: if cmd.max_linear_vel != 0.0 {
                cmd.max_linear_vel
            } else {
                default_params.max_vel_linear
            },
            max_vel_angular: if cmd.max_angular_vel != 0.0 {
                cmd.max_angular_vel
            } else {
                default_params.max_vel_angular
            },
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

        let (body_twist, body_accel) = ctx.pose_control_policy(cmd.as_vec3f(), traj_params)?;

        let telem = SkillExtendedTelemetry::GlobalPosition(ExtendedGlobalPositionTelemetry {
            cmd_echo: cmd,
        });

        Ok((SkillSetpoints { body_twist, body_accel }, telem))
    }

    fn reset(&mut self) {
        
    }
}
