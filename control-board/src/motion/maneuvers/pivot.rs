use crate::motion::control_context::{ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::{ExtendedPivotTelemetry, PivotCommand};
use ateam_common_packets::radio::ManeuverExtendedTelemetry;
use ateam_controls::pivot_trajectory::PivotParams;
use ateam_controls::ControlsError;

pub struct PivotManeuver;

impl PivotManeuver {
    pub fn new() -> Self {
        Self
    }
}

impl MotionManeuver for PivotManeuver {
    type Command = PivotCommand;

    fn entry(&mut self, _cmd: PivotCommand, _ctx: &mut ControlContext) {}

    fn update(
        &mut self,
        cmd: PivotCommand,
        ctx: &mut ControlContext,
    ) -> Result<(ManeuverSetpoints, ManeuverExtendedTelemetry), ControlsError> {
        let default_params = PivotParams::default();

        let traj_params = PivotParams {
            max_vel_angular: if cmd.max_angular_vel != 0.0 {
                cmd.max_angular_vel
            } else {
                default_params.max_vel_angular
            },
            max_accel_angular: if cmd.max_angular_acc != 0.0 {
                cmd.max_angular_acc
            } else {
                default_params.max_accel_angular
            },
            orbit_radius: if cmd.orbit_radius != 0.0 {
                cmd.orbit_radius
            } else {
                default_params.orbit_radius
            },
            heading_lag: cmd.heading_lag,
        };

        let (body_twist, body_accel) = ctx.pivot_control_policy(cmd.center(), cmd.global_theta, traj_params)?;

        let telem = ManeuverExtendedTelemetry::Pivot(ExtendedPivotTelemetry {
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

