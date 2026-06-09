use crate::motion::control_context::{CommandFrame, ControlContext, SkillSetpoints};
use crate::motion::skills::MotionSkill;
use ateam_common_packets::bindings::{ExtendedLocalAccelerationTelemetry, LocalAccelerationCommand};
use ateam_common_packets::radio::SkillExtendedTelemetry;
use ateam_controls::ControlsError;

pub struct LocalAccelerationSkill;

impl LocalAccelerationSkill {
    pub fn new() -> Self {
        Self
    }
}

impl MotionSkill for LocalAccelerationSkill {
    type Command = LocalAccelerationCommand;

    fn entry(&mut self, _cmd: LocalAccelerationCommand, _ctx: &mut ControlContext) {

    }

    fn update(
        &mut self,
        cmd: LocalAccelerationCommand,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError> {
        let (body_twist, body_accel) = ctx.accel_control_policy(cmd.as_vec3f(), CommandFrame::Local);

        let telem = SkillExtendedTelemetry::LocalAcceleration(ExtendedLocalAccelerationTelemetry {
            cmd_echo: cmd,
        });
        
        Ok((SkillSetpoints { body_twist, body_accel }, telem))
    }

    fn reset(&mut self) {}
}
