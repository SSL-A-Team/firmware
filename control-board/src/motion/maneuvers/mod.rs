pub mod global_acceleration;
pub mod global_position;
pub mod global_velocity;
pub mod local_acceleration;
pub mod local_velocity;
pub mod pivot;

use crate::motion::control_context::ControlContext;
pub use crate::motion::control_context::ManeuverSetpoints;
use ateam_common_packets::bindings::{BasicControl, BodyControlMode};
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::ControlsError;

use crate::motion::maneuvers::global_acceleration::GlobalAccelerationManeuver;
use crate::motion::maneuvers::global_position::GlobalPositionManeuver;
use crate::motion::maneuvers::global_velocity::GlobalVelocityManeuver;
use crate::motion::maneuvers::local_acceleration::LocalAccelerationManeuver;
use crate::motion::maneuvers::local_velocity::LocalVelocityManeuver;
use crate::motion::maneuvers::pivot::PivotManeuver;

pub trait MotionManeuver {
    type Command: Copy;

    fn entry(&mut self, cmd: Self::Command, ctx: &mut ControlContext);

    fn update(
        &mut self,
        cmd: Self::Command,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError>;

    fn reset(&mut self);
}

enum ActiveManeuver {
    Off,
    GlobalPosition(GlobalPositionManeuver),
    GlobalVelocity(GlobalVelocityManeuver),
    LocalVelocity(LocalVelocityManeuver),
    GlobalAcceleration(GlobalAccelerationManeuver),
    LocalAcceleration(LocalAccelerationManeuver),
    Pivot(PivotManeuver),
}

impl ActiveManeuver {
    fn from_maneuver_command(cmd: &ManeuverCommand) -> Self {
        match cmd {
            ManeuverCommand::GlobalPosition(_) => {
                Self::GlobalPosition(GlobalPositionManeuver::new())
            }
            ManeuverCommand::GlobalVelocity(_) => {
                Self::GlobalVelocity(GlobalVelocityManeuver::new())
            }
            ManeuverCommand::LocalVelocity(_) => Self::LocalVelocity(LocalVelocityManeuver::new()),
            ManeuverCommand::GlobalAcceleration(_) => {
                Self::GlobalAcceleration(GlobalAccelerationManeuver::new())
            }
            ManeuverCommand::LocalAcceleration(_) => {
                Self::LocalAcceleration(LocalAccelerationManeuver::new())
            }
            ManeuverCommand::Pivot(_) => Self::Pivot(PivotManeuver::new()),
            ManeuverCommand::Off => Self::Off,
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
            Self::Pivot(s) => s.reset(),
        }
    }

    fn entry_cmd(&mut self, cmd: &ManeuverCommand, ctx: &mut ControlContext) {
        match (self, cmd) {
            (Self::GlobalPosition(s), ManeuverCommand::GlobalPosition(c)) => s.entry(*c, ctx),
            (Self::GlobalVelocity(s), ManeuverCommand::GlobalVelocity(c)) => s.entry(*c, ctx),
            (Self::LocalVelocity(s), ManeuverCommand::LocalVelocity(c)) => s.entry(*c, ctx),
            (Self::GlobalAcceleration(s), ManeuverCommand::GlobalAcceleration(c)) => {
                s.entry(*c, ctx)
            }
            (Self::LocalAcceleration(s), ManeuverCommand::LocalAcceleration(c)) => s.entry(*c, ctx),
            (Self::Pivot(s), ManeuverCommand::Pivot(c)) => s.entry(*c, ctx),
            _ => {}
        }
    }

    fn update_cmd(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        match (self, cmd) {
            (Self::GlobalPosition(s), ManeuverCommand::GlobalPosition(c)) => s.update(c, ctx),
            (Self::GlobalVelocity(s), ManeuverCommand::GlobalVelocity(c)) => s.update(c, ctx),
            (Self::LocalVelocity(s), ManeuverCommand::LocalVelocity(c)) => s.update(c, ctx),
            (Self::GlobalAcceleration(s), ManeuverCommand::GlobalAcceleration(c)) => {
                s.update(c, ctx)
            }
            (Self::LocalAcceleration(s), ManeuverCommand::LocalAcceleration(c)) => s.update(c, ctx),
            (Self::Pivot(s), ManeuverCommand::Pivot(c)) => s.update(c, ctx),
            _ => Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off)),
        }
    }
}

pub struct ManeuverManager {
    active: ActiveManeuver,
    prev_mode: BodyControlMode::Type,
}

impl ManeuverManager {
    pub fn new() -> Self {
        Self {
            active: ActiveManeuver::Off,
            prev_mode: BodyControlMode::BCM_OFF,
        }
    }

    pub fn reset(&mut self) {
        self.active.reset();
        self.active = ActiveManeuver::Off;
        self.prev_mode = BodyControlMode::BCM_OFF;
    }

    pub fn tick(
        &mut self,
        cmd: BasicControl,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let maneuver_cmd = cmd.get_maneuver_command();
        if cmd.body_control_mode != self.prev_mode {
            self.prev_mode = cmd.body_control_mode;
            self.active.reset();
            self.active = ActiveManeuver::from_maneuver_command(&maneuver_cmd);
            self.active.entry_cmd(&maneuver_cmd, ctx);
        }
        self.active.update_cmd(maneuver_cmd, ctx)
    }
}

impl Default for ManeuverManager {
    fn default() -> Self {
        Self::new()
    }
}
