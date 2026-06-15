pub mod global_acceleration;
pub mod global_position;
pub mod global_velocity;
pub mod local_acceleration;
pub mod local_velocity;

use crate::motion::control_context::ControlContext;
pub use crate::motion::control_context::SkillSetpoints;
use ateam_common_packets::bindings::{BasicControl, BodyControlMode};
use ateam_common_packets::radio::{SkillCommand, SkillExtendedTelemetry};
use ateam_controls::ControlsError;

use crate::motion::skills::global_acceleration::GlobalAccelerationSkill;
use crate::motion::skills::global_position::GlobalPositionSkill;
use crate::motion::skills::global_velocity::GlobalVelocitySkill;
use crate::motion::skills::local_acceleration::LocalAccelerationSkill;
use crate::motion::skills::local_velocity::LocalVelocitySkill;

pub trait MotionSkill {
    type Command: Copy;

    fn entry(&mut self, cmd: Self::Command, ctx: &mut ControlContext);

    fn update(
        &mut self,
        cmd: Self::Command,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError>;

    fn reset(&mut self);
}

enum ActiveSkill {
    Off,
    GlobalPosition(GlobalPositionSkill),
    GlobalVelocity(GlobalVelocitySkill),
    LocalVelocity(LocalVelocitySkill),
    GlobalAcceleration(GlobalAccelerationSkill),
    LocalAcceleration(LocalAccelerationSkill),
}

impl ActiveSkill {
    fn from_skill_command(cmd: &SkillCommand) -> Self {
        match cmd {
            SkillCommand::GlobalPosition(_) => Self::GlobalPosition(GlobalPositionSkill::new()),
            SkillCommand::GlobalVelocity(_) => Self::GlobalVelocity(GlobalVelocitySkill::new()),
            SkillCommand::LocalVelocity(_) => Self::LocalVelocity(LocalVelocitySkill::new()),
            SkillCommand::GlobalAcceleration(_) => {
                Self::GlobalAcceleration(GlobalAccelerationSkill::new())
            }
            SkillCommand::LocalAcceleration(_) => {
                Self::LocalAcceleration(LocalAccelerationSkill::new())
            }
            SkillCommand::Off => Self::Off,
        }
    }

    fn reset(&mut self) {
        match self {
            Self::Off => {}
            Self::GlobalPosition(s) => s.reset(),
            Self::GlobalVelocity(s) => s.reset(),
            Self::LocalVelocity(s) => s.reset(),
            Self::GlobalAcceleration(s) => s.reset(),
            Self::LocalAcceleration(s) => s.reset(),
        }
    }

    fn entry_cmd(&mut self, cmd: &SkillCommand, ctx: &mut ControlContext) {
        match (self, cmd) {
            (Self::GlobalPosition(s), SkillCommand::GlobalPosition(c)) => s.entry(*c, ctx),
            (Self::GlobalVelocity(s), SkillCommand::GlobalVelocity(c)) => s.entry(*c, ctx),
            (Self::LocalVelocity(s), SkillCommand::LocalVelocity(c)) => s.entry(*c, ctx),
            (Self::GlobalAcceleration(s), SkillCommand::GlobalAcceleration(c)) => s.entry(*c, ctx),
            (Self::LocalAcceleration(s), SkillCommand::LocalAcceleration(c)) => s.entry(*c, ctx),
            _ => {}
        }
    }

    fn update_cmd(
        &mut self,
        cmd: SkillCommand,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError> {
        match (self, cmd) {
            (Self::GlobalPosition(s), SkillCommand::GlobalPosition(c)) => s.update(c, ctx),
            (Self::GlobalVelocity(s), SkillCommand::GlobalVelocity(c)) => s.update(c, ctx),
            (Self::LocalVelocity(s), SkillCommand::LocalVelocity(c)) => s.update(c, ctx),
            (Self::GlobalAcceleration(s), SkillCommand::GlobalAcceleration(c)) => s.update(c, ctx),
            (Self::LocalAcceleration(s), SkillCommand::LocalAcceleration(c)) => s.update(c, ctx),
            _ => Ok((SkillSetpoints::zero(), SkillExtendedTelemetry::Off)),
        }
    }
}

pub struct SkillManager {
    active: ActiveSkill,
    prev_mode: BodyControlMode::Type,
}

impl SkillManager {
    pub fn new() -> Self {
        Self {
            active: ActiveSkill::Off,
            prev_mode: BodyControlMode::BCM_OFF,
        }
    }

    pub fn reset(&mut self) {
        self.active.reset();
        self.active = ActiveSkill::Off;
        self.prev_mode = BodyControlMode::BCM_OFF;
    }

    pub fn tick(
        &mut self,
        cmd: BasicControl,
        ctx: &mut ControlContext,
    ) -> Result<(SkillSetpoints, SkillExtendedTelemetry), ControlsError> {
        let skill_cmd = cmd.get_skill_command();
        if cmd.body_control_mode != self.prev_mode {
            self.prev_mode = cmd.body_control_mode;
            self.active.reset();
            self.active = ActiveSkill::from_skill_command(&skill_cmd);
            self.active.entry_cmd(&skill_cmd, ctx);
        }
        self.active.update_cmd(skill_cmd, ctx)
    }
}

impl Default for SkillManager {
    fn default() -> Self {
        Self::new()
    }
}
