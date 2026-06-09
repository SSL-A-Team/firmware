use crate::motion::control_context::{CommandFrame, ControlContext, SkillSetpoints};
use crate::motion::skills::MotionSkill;
use ateam_common_packets::bindings::{ExtendedGlobalAccelerationTelemetry, GlobalAccelerationCommand};
use ateam_common_packets::radio::SkillExtendedTelemetry;
use ateam_controls::ControlsError;

pub struct GlobalAccelerationSkill;

impl GlobalAccelerationSkill {
    pub fn new() -> Self {
        Self
    }
}

impl MotionSkill for GlobalAccelerationSkill {
    type Command = GlobalAccelerationCommand;

    fn entry(&mut self, _cmd: GlobalAccelerationCommand, _ctx: &mut ControlContext) {

    }

    fn update(
        &mut self,
        cmd: GlobalAccelerationCommand,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError> {
        let (body_twist, body_accel) = ctx.accel_control_policy(cmd.as_vec3f(), CommandFrame::Global);

        let telem = SkillExtendedTelemetry::GlobalAcceleration(ExtendedGlobalAccelerationTelemetry {
            cmd_echo: cmd,
        });

        Ok((SkillSetpoints { body_twist, body_accel }, telem))
    }

    fn reset(&mut self) {
        
    }
}
