use embassy_time::Duration;
use libm::powf;
use nalgebra::SVector;

/// Physically-identified parameters for a [`FirstOrderLag`] model.
///
/// These are determined experimentally by comparing commanded and measured
/// velocity across operating points, and are independent of control design choices.
pub struct FirstOrderLagParams<const N: usize> {
    /// DC gain per axis.
    pub k: SVector<f32, N>,
    /// Slope of T_eff vs. input magnitude per axis [s / (m/s)]: dT_eff / d|u|.
    pub t_slope: SVector<f32, N>,
}

/// First-order lag element with velocity-dependent time constant.
///
/// Models the nonlinear first-order ODE:
///
/// ```text
/// T_eff(u) = t_slope * |u| + t_min
/// dv/dt    = (K / T_eff) * u  -  (1 / T_eff) * v
/// ```
///
/// `N` is the number of independent axes, each with its own `k` and `t_slope`.
/// The `t_min` floor, timestep `dt`, and inversion step count are shared across axes
/// and fixed at construction time.
pub struct FirstOrderLag<const N: usize> {
    /// Physically-identified model parameters.
    params: FirstOrderLagParams<N>,
    /// Minimum time constant floor, shared across axes [s].
    t_min: f32,
    /// Integration timestep [s].
    dt: f32,
    /// Fixed-point iterations used by [`Self::invert`].
    n_steps: usize,
    /// Integrator state (the modelled output velocity) per axis.
    state: SVector<f32, N>,
}

/// Prevents `T_eff` collapsing to zero at standstill.
const T_MIN_DEFAULT: Duration = Duration::from_millis(1);

fn duration_to_secs(d: Duration) -> f32 {
    d.as_micros() as f32 * 1e-6
}

impl<const N: usize> FirstOrderLag<N> {
    /// Create with an explicit iteration count.
    ///
    /// `t_min` defaults to 1 ms when `None`.
    pub fn new(
        params: FirstOrderLagParams<N>,
        t_min: Option<Duration>,
        dt: Duration,
        n_steps: usize,
    ) -> Self {
        Self {
            params,
            t_min: duration_to_secs(t_min.unwrap_or(T_MIN_DEFAULT)),
            dt: duration_to_secs(dt),
            n_steps,
            state: SVector::zeros(),
        }
    }

    /// Create with a time horizon; converts `t_horizon` to
    /// `n_steps = max(1, round(t_horizon / dt))` once at construction.
    ///
    /// `t_min` defaults to 1 ms when `None`.
    pub fn new_from_horizon(
        params: FirstOrderLagParams<N>,
        t_min: Option<Duration>,
        dt: Duration,
        t_horizon: Duration,
    ) -> Self {
        let dt_secs = duration_to_secs(dt);
        let n_steps = ((duration_to_secs(t_horizon) / dt_secs + 0.5) as usize).max(1);
        Self {
            params,
            t_min: duration_to_secs(t_min.unwrap_or(T_MIN_DEFAULT)),
            dt: dt_secs,
            n_steps,
            state: SVector::zeros(),
        }
    }

    pub fn state(&self) -> &SVector<f32, N> {
        &self.state
    }

    pub fn n_steps(&self) -> usize {
        self.n_steps
    }

    /// Command amplification factor at reference input magnitude `u_ref` for a given `axis`.
    ///
    /// Returns the ratio by which `invert` scales a steady-state command of magnitude
    /// `u_ref` (starting from rest). Amplification of 1.0 means no boost; larger values
    /// mean stronger pre-compensation and larger peak transient commands. Use this to
    /// characterise how aggressive the lag compensation is before enabling it.
    pub fn cmd_amplification(&self, u_ref: f32, axis: usize) -> f32 {
        let t_eff = self.params.t_slope[axis] * u_ref + self.t_min;
        let a = self.dt * self.params.k[axis] / t_eff;
        (1.0 / a) * (1.0 - powf(1.0 - a, (self.n_steps + 1) as f32))
    }

    pub fn reset(&mut self) {
        self.state = SVector::zeros();
    }

    /// Advance the model one timestep given `input` commands.
    pub fn step(&mut self, input: &SVector<f32, N>) {
        for i in 0..N {
            let t_eff = self.params.t_slope[i] * input[i].abs() + self.t_min;
            let one_over_t = 1.0 / t_eff;
            self.state[i] += one_over_t * self.dt * (self.params.k[i] * input[i] - self.state[i]);
        }
    }

    /// Invert the model: find the input that drives the state to `desired` in one step.
    ///
    /// Uses a fixed-point iteration with `T_eff` frozen from `desired` (linearising
    /// approximation). The inner loop is pre-collapsed to the closed-form recurrence:
    ///
    /// ```text
    /// cmd_{j+1} = (1 - a) * cmd_j  +  (desired - state * (1 - b))
    /// ```
    ///
    /// where `a = dt * K / T_eff` and `b = dt / T_eff` are loop-invariant, so
    /// only one division and two multiplies are needed per axis outside the loop.
    pub fn invert(&self, desired: &SVector<f32, N>) -> SVector<f32, N> {
        let mut output = SVector::zeros();
        for i in 0..N {
            let t_eff = self.params.t_slope[i] * desired[i].abs() + self.t_min;
            let one_over_t = 1.0 / t_eff;
            let a = self.dt * self.params.k[i] * one_over_t;
            let target = desired[i] - self.state[i] * (1.0 - self.dt * one_over_t);
            let one_minus_a = 1.0 - a;
            let mut cmd = desired[i];
            for _ in 0..self.n_steps {
                cmd = one_minus_a * cmd + target;
            }
            output[i] = cmd;
        }
        output
    }
}
