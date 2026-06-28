//! Shared helpers for the heading- and point-based pivot maneuvers.

use ateam_common_packets::bindings::PivotDirection;
use ateam_controls::pivot_trajectory::{PivotDirection as TrajPivotDirection, PivotParams};

/// Build [`PivotParams`] from the fields common to both pivot commands, falling
/// back to the per-field defaults when a limit is left at zero.
pub(super) fn build_pivot_params(
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
