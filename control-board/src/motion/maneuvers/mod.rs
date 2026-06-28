pub mod global_acceleration;
pub mod global_position;
pub mod global_velocity;
pub mod heading_line;
pub mod heading_pivot;
pub mod local_acceleration;
pub mod local_velocity;
pub mod point_line;
pub mod point_pivot;

use crate::motion::control_context::ControlContext;
pub use crate::motion::control_context::ManeuverSetpoints;
use ateam_common_packets::bindings::{BasicControl, BodyControlMode, PivotDirection};
use ateam_common_packets::radio::{ManeuverCommand, ManeuverExtendedTelemetry};
use ateam_controls::linear_trajectory::LinearParams;
use ateam_controls::pivot_trajectory::{PivotDirection as TrajPivotDirection, PivotParams};
use ateam_controls::ControlsError;

use crate::motion::maneuvers::global_acceleration::GlobalAccelerationManeuver;
use crate::motion::maneuvers::global_position::GlobalPositionManeuver;
use crate::motion::maneuvers::global_velocity::GlobalVelocityManeuver;
use crate::motion::maneuvers::heading_line::HeadingLineManeuver;
use crate::motion::maneuvers::heading_pivot::HeadingPivotManeuver;
use crate::motion::maneuvers::local_acceleration::LocalAccelerationManeuver;
use crate::motion::maneuvers::local_velocity::LocalVelocityManeuver;
use crate::motion::maneuvers::point_line::PointLineManeuver;
use crate::motion::maneuvers::point_pivot::PointPivotManeuver;

/// Build [`PivotParams`] from the fields common to both pivot commands, falling
/// back to the per-field defaults when a limit is left at zero.
pub(crate) fn build_pivot_params(
    max_angular_vel: f32,
    max_angular_acc: f32,
    orbit_radius: f32,
    inset_angle: f32,
    direction: PivotDirection::Type,
    compute_inset_angle: u8,
) -> PivotParams {
    let default_params = PivotParams::default();
    let direction = if direction == PivotDirection::PIVOT_DIRECTION_BACKWARD {
        TrajPivotDirection::Backward
    } else {
        TrajPivotDirection::Forward
    };
    PivotParams {
        max_vel_angular: if max_angular_vel != 0.0 {
            max_angular_vel
        } else {
            default_params.max_vel_angular
        },
        max_accel_angular: if max_angular_acc != 0.0 {
            max_angular_acc
        } else {
            default_params.max_accel_angular
        },
        orbit_radius: if orbit_radius != 0.0 {
            orbit_radius
        } else {
            default_params.orbit_radius
        },
        inset_angle,
        compute_inset_angle: compute_inset_angle != 0,
        direction,
    }
}

/// Build [`LinearParams`] from the limit fields common to both line commands,
/// falling back to the per-field defaults when a limit is left at zero.
pub(crate) fn build_linear_params(
    max_vel_colinear: f32,
    max_vel_perp: f32,
    max_vel_angular: f32,
    max_accel_colinear: f32,
    max_accel_perp: f32,
    max_accel_angular: f32,
    colinear_start_thresh: f32,
) -> LinearParams {
    let d = LinearParams::default();
    LinearParams {
        max_vel_colinear: if max_vel_colinear != 0.0 {
            max_vel_colinear
        } else {
            d.max_vel_colinear
        },
        max_vel_perp: if max_vel_perp != 0.0 {
            max_vel_perp
        } else {
            d.max_vel_perp
        },
        max_vel_angular: if max_vel_angular != 0.0 {
            max_vel_angular
        } else {
            d.max_vel_angular
        },
        max_accel_colinear: if max_accel_colinear != 0.0 {
            max_accel_colinear
        } else {
            d.max_accel_colinear
        },
        max_accel_perp: if max_accel_perp != 0.0 {
            max_accel_perp
        } else {
            d.max_accel_perp
        },
        max_accel_angular: if max_accel_angular != 0.0 {
            max_accel_angular
        } else {
            d.max_accel_angular
        },
        colinear_start_thresh_linear: if colinear_start_thresh != 0.0 {
            colinear_start_thresh
        } else {
            d.colinear_start_thresh_linear
        },
    }
}

pub trait MotionManeuver {
    fn entry(&mut self, cmd: ManeuverCommand, ctx: &mut ControlContext);

    fn update(
        &mut self,
        cmd: ManeuverCommand,
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
    HeadingPivot(HeadingPivotManeuver),
    PointPivot(PointPivotManeuver),
    HeadingLine(HeadingLineManeuver),
    PointLine(PointLineManeuver),
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
            ManeuverCommand::HeadingPivot(_) => Self::HeadingPivot(HeadingPivotManeuver::new()),
            ManeuverCommand::PointPivot(_) => Self::PointPivot(PointPivotManeuver::new()),
            ManeuverCommand::HeadingLine(_) => Self::HeadingLine(HeadingLineManeuver::new()),
            ManeuverCommand::PointLine(_) => Self::PointLine(PointLineManeuver::new()),
            ManeuverCommand::Off => Self::Off,
            _ => {
                Self::Off
            },
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
            Self::HeadingPivot(s) => s.reset(),
            Self::PointPivot(s) => s.reset(),
            Self::HeadingLine(s) => s.reset(),
            Self::PointLine(s) => s.reset(),
        }
    }

    fn entry_cmd(&mut self, cmd: ManeuverCommand, ctx: &mut ControlContext) {
        match self {
            Self::Off => {}
            Self::GlobalPosition(s) => s.entry(cmd, ctx),
            Self::GlobalVelocity(s) => s.entry(cmd, ctx),
            Self::LocalVelocity(s) => s.entry(cmd, ctx),
            Self::GlobalAcceleration(s) => s.entry(cmd, ctx),
            Self::LocalAcceleration(s) => s.entry(cmd, ctx),
            Self::HeadingPivot(s) => s.entry(cmd, ctx),
            Self::PointPivot(s) => s.entry(cmd, ctx),
            Self::HeadingLine(s) => s.entry(cmd, ctx),
            Self::PointLine(s) => s.entry(cmd, ctx),
        }
    }

    fn update_cmd(
        &mut self,
        cmd: ManeuverCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        match self {
            Self::Off => Ok((ManeuverSetpoints::zero(), ManeuverExtendedTelemetry::Off)),
            Self::GlobalPosition(s) => s.update(cmd, ctx),
            Self::GlobalVelocity(s) => s.update(cmd, ctx),
            Self::LocalVelocity(s) => s.update(cmd, ctx),
            Self::GlobalAcceleration(s) => s.update(cmd, ctx),
            Self::LocalAcceleration(s) => s.update(cmd, ctx),
            Self::HeadingPivot(s) => s.update(cmd, ctx),
            Self::PointPivot(s) => s.update(cmd, ctx),
            Self::HeadingLine(s) => s.update(cmd, ctx),
            Self::PointLine(s) => s.update(cmd, ctx),
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
            ctx.reset_trajectory();
            self.active = ActiveManeuver::from_maneuver_command(&maneuver_cmd);
            self.active.entry_cmd(maneuver_cmd, ctx);
        }
        self.active.update_cmd(maneuver_cmd, ctx)
    }
}

impl Default for ManeuverManager {
    fn default() -> Self {
        Self::new()
    }
}
