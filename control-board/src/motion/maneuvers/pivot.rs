use crate::motion::control_context::{ControlContext, ManeuverSetpoints};
use crate::motion::maneuvers::MotionManeuver;
use ateam_common_packets::bindings::{ExtendedPivotTelemetry, PivotCommand};
use ateam_common_packets::radio::ManeuverExtendedTelemetry;
use ateam_controls::pivot_trajectory::PivotParams;
use ateam_controls::ControlsError;

pub struct PivotManeuver {
    prev_cmd: Option<PivotCommand>,
}

impl PivotManeuver {
    pub fn new() -> Self {
        Self { prev_cmd: None }
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

        // Only force a replan when the command values actually change.
        let force_replan = match &self.prev_cmd {
            None => true,
            Some(prev) => {
                prev.global_x_center != cmd.global_x_center
                    || prev.global_y_center != cmd.global_y_center
                    || prev.global_theta != cmd.global_theta
                    || prev.max_angular_vel != cmd.max_angular_vel
                    || prev.max_angular_acc != cmd.max_angular_acc
                    || prev.orbit_radius != cmd.orbit_radius
                    || prev.heading_lag != cmd.heading_lag
            }
        };
        self.prev_cmd = Some(cmd);

        let (body_twist, body_accel) =
            ctx.pivot_control_policy(cmd.center(), cmd.global_theta, traj_params, force_replan)?;

        let telem = ManeuverExtendedTelemetry::Pivot(ExtendedPivotTelemetry { cmd_echo: cmd });

        Ok((
            ManeuverSetpoints {
                body_twist,
                body_accel,
            },
            telem,
        ))
    }

    fn reset(&mut self) {
        self.prev_cmd = None;
    }
}
