//! Per-wheel thermal current budget for the feed-forward output stage.
//!
//! Drive motor: Nanotec DF45M024053-A2 (50 W): 2.36 A rated continuous, 7 A peak
//! (0.084 / 0.25 N*m). The motor firmware clamps to 2.16 A (90% of rated) once its
//! I2T budget is spent and to 6.3 A (90% of peak) otherwise.
//!
//! Mirrors the wheel motor firmware's I2T protection (motor-controller/bin/
//! wheel-torque: budget CURRENT_I2T_BUDGET_A2S above MAX_CURR_WHEEL_NOT_TURNING,
//! after which the motor hard-clamps to the continuous current), using the
//! applied currents the motors report. The model runs on a smaller budget than
//! the motor's so the body controller derates first and smoothly, keeping the
//! acceleration scaling at the body level (direction preserved) instead of one
//! wheel being clipped by its motor mid-move. The motor's own I2T stays as the
//! independent hard limit.

use ateam_controls::Vector4f;

/// Continuous wheel current the motor firmware falls back to (A), MAX_CURR_WHEEL_NOT_TURNING.
pub const MOTOR_CONTINUOUS_CURRENT_A: f32 = 2.16;
/// Motor firmware I2T budget above the continuous current (A^2*s), CURRENT_I2T_BUDGET_A2S.
pub const MOTOR_I2T_BUDGET_A2S: f32 = 11.3;
/// Fraction of the motor's budget the control board allows itself.
const BUDGET_FRACTION: f32 = 0.8;
/// Fraction of the (control board) budget used before derating starts.
const DERATE_START_FRACTION: f32 = 0.75;
/// Feed-forward current once derated (A): below the motor's continuous clamp so the
/// wheel velocity loop keeps some authority.
pub const FF_CONTINUOUS_CURRENT_A: f32 = 1.9;

pub struct WheelThermalBudget {
    used_a2s: Vector4f,
}

impl WheelThermalBudget {
    pub const fn new() -> Self {
        Self {
            used_a2s: Vector4f::new(0.0, 0.0, 0.0, 0.0),
        }
    }

    /// Integrate the applied wheel currents (A) over `dt` seconds.
    pub fn update(&mut self, applied_currents_a: &Vector4f, dt: f32) {
        let budget = BUDGET_FRACTION * MOTOR_I2T_BUDGET_A2S;
        let cont_sq = MOTOR_CONTINUOUS_CURRENT_A * MOTOR_CONTINUOUS_CURRENT_A;
        for i in 0..4 {
            let i_a = applied_currents_a[i];
            self.used_a2s[i] = (self.used_a2s[i] + (i_a * i_a - cont_sq) * dt).clamp(0.0, budget);
        }
    }

    /// Per-wheel feed-forward current limits (A): `peak_a` while the budget lasts,
    /// ramping linearly down to the continuous feed-forward current over the last
    /// part of the budget.
    pub fn ff_limits(&self, peak_a: f32) -> Vector4f {
        let budget = BUDGET_FRACTION * MOTOR_I2T_BUDGET_A2S;
        let start = DERATE_START_FRACTION * budget;
        let cont = FF_CONTINUOUS_CURRENT_A.min(peak_a);
        let mut limits = Vector4f::zeros();
        for i in 0..4 {
            let frac = ((self.used_a2s[i] - start) / (budget - start)).clamp(0.0, 1.0);
            limits[i] = peak_a + (cont - peak_a) * frac;
        }
        limits
    }

    pub fn used_fraction(&self) -> Vector4f {
        self.used_a2s / (BUDGET_FRACTION * MOTOR_I2T_BUDGET_A2S)
    }
}
