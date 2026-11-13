use crate::motion::params::controller_params::{
    ENC_LAG_K, ENC_LAG_MODE, ENC_LAG_T_HORIZON, ENC_LAG_T_SLOPE, FF_ACCEL_LOOKAHEAD_S, TRAJ_RESET_ERROR_XY,
    TRACKING_DIVERGENCE_RECOVERY_REST_TICKS,
    TRACKING_DIVERGENCE_RECOVERY_REST_WHEEL_VEL,
};
use crate::motion::accel_trajectory::AccelTrajectory;
use crate::motion::velocity_ramp::{VelocityFrame, VelocityRamp};
use ateam_common_packets::bindings::{ParameterCommand, ParameterDataFormat, ParameterName};
use ateam_common_packets::radio::ManeuverCommand;
use ateam_controls::defaults::{DEFAULT_EKF_BUFF_LEN, DEFAULT_VISION_BUFF_LEN, EKF_INPUT_LEN, EKF_MEAS_LEN, EKF_STATE_LEN};
use ateam_controls::state_estimation::{BufferedEKF, Odometer, StateEstimator, VisionFilter, VisionSample};
use ateam_controls::bangbang_trajectory::BangBangTraj3D;
use ateam_controls::linear_trajectory::LinearTrajectory;
use ateam_controls::pivot_trajectory::PivotTrajectory;
use ateam_controls::robot_model::{RobotModel, RobotPhysicalParams};
use ateam_controls::trajectory::Trajectory;
use ateam_controls::{
    ControlsError, Vector3f, Vector4f, Vector6f, Vector8f,
};
use ateam_lib_stm32::model::{FirstOrderLag, FirstOrderLagParams};
use core::f32::consts::PI;
use embassy_time::{Duration, Instant};
use libm::{fabsf, hypotf, remainderf, sqrtf};
use nalgebra::{SMatrix, SVector};

pub(crate) const VISION_ACTIVE_TIMEOUT_S: f32 = 0.5;

/// Fraction of the feasible acceleration planners may use (rest is feedback headroom).
/// 0.9 with the identified efficiency (0.75): same time-to-target as 0.8 with the
/// optimistic 0.85, with ~25% less tracking error.
pub const FEASIBLE_ACCEL_MARGIN: f32 = 0.9;
/// Floor for the planned linear acceleration (m/s^2) so a fully derated robot still moves.
pub const MIN_PLANNED_ACCEL_LINEAR: f32 = 0.3;

/// All trajectory types the firmware can track.
///
/// Provides static dispatch over trajectory types that each implement the
/// `Trajectory` trait directly (owning their own state).
pub enum TrackedTrajectory {
    BangBang(BangBangTraj3D),
    Pivot(PivotTrajectory),
    Linear(LinearTrajectory),
    Accel(AccelTrajectory),
    VelocityRamp(VelocityRamp),
}

impl Trajectory for TrackedTrajectory {
    fn tick(&mut self, dt: f32) {
        match self {
            Self::BangBang(t) => t.tick(dt),
            Self::Pivot(t) => t.tick(dt),
            Self::Linear(t) => t.tick(dt),
            Self::Accel(t) => t.tick(dt),
            Self::VelocityRamp(t) => t.tick(dt),
        }
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        match self {
            Self::BangBang(t) => t.sample(),
            Self::Pivot(t) => t.sample(),
            Self::Linear(t) => t.sample(),
            Self::Accel(t) => t.sample(),
            Self::VelocityRamp(t) => t.sample(),
        }
    }
}

/// Body-frame setpoints produced by a maneuver each tick.
/// `body_controller` applies friction compensation and wheel transforms.
#[derive(Copy, Clone, Default)]
pub struct ManeuverSetpoints {
    pub body_twist: Vector3f,
    pub body_accel: Vector3f,
}

impl ManeuverSetpoints {
    pub fn zero() -> Self {
        Self {
            body_twist: Vector3f::zeros(),
            body_accel: Vector3f::zeros(),
        }
    }
}

/// Trajectory-divergence recovery state. A large unexpected tracking error
/// (e.g. a collision knocks the robot off course) trips the controller into
/// `Recovering`, where it commands an active brake until the wheels stop; only
/// then is the controller reset and normal tracking resumed.
#[derive(PartialEq, Eq, Clone, Copy, Default)]
pub enum TrackingDivergenceState {
    /// Normal operation; monitoring trajectory tracking for divergence.
    #[default]
    Normal,
    /// Diverged: commanding active brake, waiting for the wheels to stop before
    /// resetting the controller.
    Recovering,
}

/// Controller infrastructure passed into each maneuver on every tick.
///
/// Owns the KF/robot-model, trajectory state, PID, and encoder-lag model.
/// Maneuvers borrow this via `&mut ControlContext` to call planning and
/// tracking helpers.
pub struct ControlContext {
    pub robot_model: RobotModel,
    pub state_estimator: StateEstimator<DEFAULT_EKF_BUFF_LEN, DEFAULT_VISION_BUFF_LEN>,
    /// XY position feedback [K, MAX, ANTI_JITTER] (see controller_params::POSE_FB_XY)
    pub pose_fb_xy: Vector3f,
    /// Angular feedback [POS_K, POS_MAX, ANTI_JITTER, VEL_K] (see controller_params::POSE_FB_W)
    pub pose_fb_w: Vector4f,
    /// Wheel velocity loop [KP, KI, ANTI_JITTER, MAX_CURRENT], forwarded to the motor controllers
    pub wheel_vel_fb: Vector4f,
    /// Feed-forward wheel current limit (A)
    pub wheel_ff_current_limit: f32,
    /// Wheel slip feed-forward ((rad/s)/A, see controller_params::WHEEL_SLIP_COMP)
    pub wheel_slip_comp: f32,
    /// Effective per-wheel feed-forward current limits (A) after thermal derating;
    /// set by the control task each tick from `WheelThermalBudget`.
    pub wheel_ff_current_limits: Vector4f,
    /// Divergence thresholds: [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
    pub tracking_divergence_thresh: Vector4f,
    /// Active trajectory. t=0 is always "now" (updated via tick each control tick).
    pub trajectory: Option<TrackedTrajectory>,
    /// Last command stored by `run_traj_track`, used to detect command changes.
    pub prev_cmd: Option<ManeuverCommand>,
    /// True while the trajectory is driven open loop (`run_traj_open_loop`). Its
    /// state is not corrected toward the estimate, so it is excluded from
    /// divergence detection and not used to seed closed-loop tracking.
    pub trajectory_open_loop: bool,
    pub enc_lag: FirstOrderLag<2>,
    pub dt: f32,
    /// Cached KF state estimate (global frame) — updated each tick before maneuver dispatch.
    pub state_estimate: Vector6f,
    /// State-estimate velocity in the robot's local frame [vx, vy, w], for telemetry.
    pub state_estimate_vel_local: Vector3f,
    /// Divergence-recovery state machine. `Recovering` engages active braking and
    /// holds it until the wheels stop, then resets the trajectory.
    pub tracking_divergence_state: TrackingDivergenceState,
    /// Consecutive ticks the wheels have been near rest while recovering.
    pub tracking_recovery_at_rest_ticks: u32,
    /// Vision state on the previous tick, to reset the trajectory when vision
    /// comes (back) online and the estimate jumps to it.
    last_vision_active: bool,
    /// Low-passed yaw rate (rad/s) for the angular rate feedback (YAW_RATE_FB_LPF_HZ).
    yaw_rate_fb: f32,
    yaw_rate_fb_alpha: f32,
}

impl ControlContext {
    pub fn new(dt: f32) -> Self {
        use crate::motion::params::controller_params;

        let enc_lag = FirstOrderLag::new_from_horizon(
            FirstOrderLagParams {
                k: SVector::<f32, 2>::new(ENC_LAG_K[0], ENC_LAG_K[1]),
                t_slope: SVector::<f32, 2>::new(ENC_LAG_T_SLOPE[0], ENC_LAG_T_SLOPE[1]),
            },
            None,
            Duration::from_micros((dt * 1e6) as u64),
            ENC_LAG_T_HORIZON,
        );

        defmt::info!(
            "enc lag: n_steps={}, cmd amplification @ 1 m/s: {}x, @ 5 m/s: {}x",
            enc_lag.n_steps(),
            enc_lag.cmd_amplification(1.0, 0),
            enc_lag.cmd_amplification(5.0, 0),
        );

        Self {
            robot_model: RobotModel::new(RobotPhysicalParams::default())
                .expect("Failed to create RobotModel, check that parameters are valid"),
            state_estimator: StateEstimator::<DEFAULT_EKF_BUFF_LEN, DEFAULT_VISION_BUFF_LEN>::default(),
            pose_fb_xy: controller_params::POSE_FB_XY,
            pose_fb_w: controller_params::POSE_FB_W,
            wheel_vel_fb: controller_params::WHEEL_VEL_FB,
            wheel_ff_current_limit: controller_params::WHEEL_FF_CURRENT_LIMIT_A,
            wheel_slip_comp: controller_params::WHEEL_SLIP_COMP,
            wheel_ff_current_limits: Vector4f::repeat(controller_params::WHEEL_FF_CURRENT_LIMIT_A),
            tracking_divergence_thresh: controller_params::TRACKING_DIVERGENCE_THRESHOLD,
            trajectory: None,
            prev_cmd: None,
            trajectory_open_loop: false,
            enc_lag,
            dt,
            state_estimate: Vector6f::zeros(),
            state_estimate_vel_local: Vector3f::zeros(),
            tracking_divergence_state: TrackingDivergenceState::Normal,
            tracking_recovery_at_rest_ticks: 0,
            last_vision_active: false,
            yaw_rate_fb: 0.0,
            yaw_rate_fb_alpha: {
                let tau = 1.0 / (2.0 * PI * controller_params::YAW_RATE_FB_LPF_HZ);
                dt / (tau + dt)
            },
        }
    }

    pub fn vision_active(&self) -> bool {
        self.state_estimator.vision_active()
    }

    pub fn applied_vision_measurement(&self) -> Option<(Vector3f, u32, bool)> {
        self.state_estimator.applied_vision()
    }

    pub fn reset(&mut self) {
        self.state_estimator.init(
            SVector::<f32, 3>::zeros(),
            SVector::<f32, 3>::zeros(),
        );
        self.trajectory = None;
        self.prev_cmd = None;
        self.trajectory_open_loop = false;
        self.enc_lag.reset();
        self.tracking_divergence_state = TrackingDivergenceState::Normal;
        self.tracking_recovery_at_rest_ticks = 0;
    }

    /// Clear trajectory and command history without touching the PID or KF,
    /// so the next tracking maneuver replans from the current state estimate.
    pub fn reset_trajectory(&mut self) {
        self.trajectory = None;
        self.prev_cmd = None;
        self.trajectory_open_loop = false;
    }

    /// Run KF vision handling, measurement construction, and state update.
    /// Returns the EKF buffer-horizon state for telemetry: global pose, then
    /// velocity in the robot's local frame.
    pub fn update_state_estimate(
        &mut self,
        vision_pose_meas: Vector3f,
        vision_t_capture_host_us: u64,
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
        imu_accel_x_meas: f32,
        imu_accel_y_meas: f32,
    ) -> Result<Vector6f, ControlsError> {

        let imu = SVector::<f32, 3>::new(
            imu_accel_x_meas,
            imu_accel_y_meas,
            imu_gyro_theta_meas,
        );

        let sample;
        let vision = if vision_update {
            sample = VisionSample {
                meas: vision_pose_meas,
                t_capture_host_us: vision_t_capture_host_us,
            };
            Some(&sample)
        } else {
            None
        };

        let encoder_vel = self.robot_model.wheel_to_local_vel(&wheel_vel_meas);

        self.state_estimator.tick(
            Instant::now().as_micros(),
            &imu,
            &encoder_vel,
            vision,
        ).map_err(|_| ControlsError::SingularMatrix)?;

        let mut state_prediction = SVector::<f32, 6>::zeros();
        state_prediction.fixed_rows_mut::<3>(0).copy_from(&self.state_estimator.get_pos_buff());
        state_prediction.fixed_rows_mut::<3>(3).copy_from(&self.state_estimator.get_vel_buff_local());

        let mut state_est = SVector::<f32, 6>::zeros();
        state_est.fixed_rows_mut::<3>(0).copy_from(&self.state_estimator.get_pos());
        state_est.fixed_rows_mut::<3>(3).copy_from(&self.state_estimator.get_vel());
        self.state_estimate = state_est;
        self.state_estimate_vel_local = self.state_estimator.get_vel_local();

        // TIGERs `CtrlPantheraUpdate`: when vision comes online the estimate jumps
        // to it, so restart the trajectory from the new estimate instead of tracking
        // the jump.
        self.yaw_rate_fb += (self.state_estimate[5] - self.yaw_rate_fb) * self.yaw_rate_fb_alpha;

        let vision_active = self.state_estimator.vision_active();
        if vision_active && !self.last_vision_active {
            self.reset_trajectory();
        }
        self.last_vision_active = vision_active;

        Ok(state_prediction)
    }

    /// Heading used to rotate the global commanded twist/accel into the local
    /// frame for the wheel outputs: the estimate advanced by half a control
    /// period, compensating the rotation during the period (TIGERs `controlXY`).
    pub fn output_heading(&self) -> f32 {
        self.state_estimate.z + self.state_estimate[5] * self.dt * 0.5
    }

    // -----------------------------------------------------------------------
    // Trajectory management and tracking
    // -----------------------------------------------------------------------

    /// Trajectory-divergence recovery state machine, run once per control tick
    /// after the maneuver dispatch. Returns `true` while the controller is
    /// recovering, signalling the control task to command the active brake.
    ///
    /// - Normal: a large unexpected tracking error (an active trajectory whose
    ///   error exceeds `TRACKING_DIVERGENCE_THRESHOLD`, e.g. a collision knocking the
    ///   robot off course) trips into `Recovering`.
    /// - Recovering: active braking is commanded until all wheels stay below
    ///   `RECOVERY_AT_REST_WHEEL_SPEED` for `RECOVERY_AT_REST_TICKS`, then the
    ///   trajectory is reset (replanning from the current state estimate) and
    ///   normal operation resumes. Uses raw encoder wheel speeds, not the KF
    ///   estimate, since divergence implies the estimate is unreliable.
    pub fn update_tracking_divergence_recovery(&mut self, wheel_vel_meas: Vector4f) -> bool {
        match self.tracking_divergence_state {
            TrackingDivergenceState::Normal => {
                if self.trajectory.is_some()
                    && !self.trajectory_open_loop
                    && self.tracking_error_exceeded()
                {
                    self.tracking_divergence_state = TrackingDivergenceState::Recovering;
                    self.tracking_recovery_at_rest_ticks = 0;
                    defmt::warn!("tracking diverged (possible collision), braking");
                    true
                } else {
                    false
                }
            }
            TrackingDivergenceState::Recovering => {
                let at_rest = wheel_vel_meas
                    .iter()
                    .all(|w| fabsf(*w) < TRACKING_DIVERGENCE_RECOVERY_REST_WHEEL_VEL);
                if at_rest {
                    self.tracking_recovery_at_rest_ticks += 1;
                } else {
                    self.tracking_recovery_at_rest_ticks = 0;
                }
                if self.tracking_recovery_at_rest_ticks >= TRACKING_DIVERGENCE_RECOVERY_REST_TICKS {
                    // Only the trajectory: reinitializing the state estimator here
                    // would drop the vision lock, and the pose jump when vision
                    // reseeds could trip another divergence.
                    self.reset_trajectory();
                    self.tracking_divergence_state = TrackingDivergenceState::Normal;
                    self.tracking_recovery_at_rest_ticks = 0;
                    defmt::warn!("recovery complete, trajectory reset");
                    false
                } else {
                    true
                }
            }
        }
    }

    /// Returns `true` when the stored command differs from `cmd`.
    ///
    /// Always returns `true` when no previous command has been stored, ensuring
    /// a replan on the first tick.
    pub fn command_changed(&self, cmd: &ManeuverCommand) -> bool {
        match &self.prev_cmd {
            None => true,
            Some(prev) => prev != cmd,
        }
    }

    /// Plan or continue a trajectory, run PID+FF tracking, advance the
    /// trajectory by `dt`, and return body-frame setpoints.
    ///
    /// # Replan decision
    /// A new trajectory is produced by calling `make_traj(seed)` when:
    /// - there is no existing trajectory, OR
    /// - `cmd` differs from the last stored command.
    ///
    /// Tracking error does **not** trigger a replan: a large unexpected error
    /// (e.g. a collision) is handled by the divergence-recovery state machine,
    /// which brakes and then resets (clearing the trajectory).
    ///
    /// # Seed selection
    /// - An existing closed-loop trajectory → seed from its current `sample()`
    ///   state (continuous handoff). Snapping to `state_estimate` would mask
    ///   divergence from the recovery state machine.
    /// - No trajectory, or an open-loop one → seed from `state_estimate`
    ///   (snap to reality; open-loop state was never corrected).
    pub fn run_traj_track<F>(
        &mut self,
        cmd: ManeuverCommand,
        make_traj: F,
    ) -> Result<ManeuverSetpoints, ControlsError>
    where
        F: FnOnce(Vector6f) -> Result<TrackedTrajectory, ControlsError>,
    {
        // Existing closed-loop trajectory → continuous seed from its current state
        // (before this tick's advance), even if the tracking error is high. Never
        // snap to the estimate here: doing so would zero the tracking error and
        // prevent the divergence-recovery state machine from ever detecting a
        // collision.
        // TIGERs `CtrlPantheraUpdate`: restart the trajectory from the estimate when
        // the XY tracking error gets large (TIGERs: when the position feedback would
        // saturate, POS_MAX / POS_K = 0.25 m), rather than chasing a trajectory the
        // robot cannot follow. See TRAJ_RESET_ERROR_XY.
        if !self.trajectory_open_loop && self.xy_feedback_saturated() {
            self.trajectory = None;
        }
        let seed_from_traj = !self.trajectory_open_loop;
        self.plan_if_needed(cmd, seed_from_traj, make_traj)?;
        self.trajectory_open_loop = false;

        let traj = self
            .trajectory
            .as_ref()
            .ok_or(ControlsError::InvalidInput)?;
        let (traj_state, mut traj_accel) = traj.sample();

        // Advance the trajectory's internal clock by dt.
        // After this, traj.sample() returns the state at the start of the next tick.
        let dt = self.dt;
        if let Some(traj) = &mut self.trajectory {
            traj.tick(dt);
            if let TrackedTrajectory::BangBang(t) = traj {
                // Feed-forward acceleration as the mean over a short lookahead of the
                // current trajectory. Replanning every tick toward a slowly moving
                // target makes a time-optimal profile brake and re-accelerate at full
                // acceleration on alternate ticks; its instantaneous acceleration then
                // chatters at +/- the limit while the mean is the small acceleration
                // actually needed. Generalizes TIGERs' one-step mean for orientation.
                // XY only: averaging the angular profile this long blurred real
                // rotations (heading tracking 0.8 -> 2.7 deg on 90 deg moves); small
                // heading-target changes are already low-passed (OrientTargetLag).
                let mut lookahead = *t;
                lookahead.tick(FF_ACCEL_LOOKAHEAD_S - dt);
                let v_end = lookahead.sample().0;
                traj_accel.x = (v_end[3] - traj_state[3]) / FF_ACCEL_LOOKAHEAD_S;
                traj_accel.y = (v_end[4] - traj_state[4]) / FF_ACCEL_LOOKAHEAD_S;
                traj_accel.z = (t.sample().0[5] - traj_state[5]) / dt;
            } else {
                // TIGERs `TrajBangBangUpdate`: angular acceleration as the mean over
                // the step, so segment switches don't produce full-scale spikes.
                traj_accel.z = (traj.sample().0[5] - traj_state[5]) / dt;
            }
        }

        self.calc_tracking_setpoints(traj_state, traj_accel)
    }

    /// Plan or continue a trajectory and drive it open loop: the setpoints are
    /// the trajectory's own twist and acceleration, with no pose feedback.
    ///
    /// Replans under the same rule as `run_traj_track`, seeded from the current
    /// trajectory state if one exists (open or closed loop), else the estimate.
    /// Accel trajectories get the state-estimate heading each tick, so
    /// `AccelFrame::Body` commands follow the robot's estimated frame.
    pub fn run_traj_open_loop<F>(
        &mut self,
        cmd: ManeuverCommand,
        make_traj: F,
    ) -> Result<ManeuverSetpoints, ControlsError>
    where
        F: FnOnce(Vector6f) -> Result<TrackedTrajectory, ControlsError>,
    {
        self.plan_if_needed(cmd, true, make_traj)?;
        self.trajectory_open_loop = true;

        let heading = self.state_estimate.z;
        let traj = self
            .trajectory
            .as_mut()
            .ok_or(ControlsError::InvalidInput)?;
        match traj {
            TrackedTrajectory::Accel(t) => t.set_body_heading(heading),
            TrackedTrajectory::VelocityRamp(t) => t.set_body_heading(heading),
            _ => {}
        }
        let (traj_state, traj_accel) = traj.sample();
        traj.tick(self.dt);

        Ok(ManeuverSetpoints {
            body_twist: traj_state.fixed_rows::<3>(3).into(),
            body_accel: traj_accel,
        })
    }

    /// Velocity maneuvers, after TIGERs `DRIVE_MODE_LOCAL_VEL` (ctrl_panthera.c
    /// `controlXY` / `controlW`): ramp toward `target` (in `frame`) at
    /// `max_accel_linear` / `max_accel_angular` and
    /// output the ramp's velocity and acceleration. No position feedback: only P
    /// on the angular rate error (`POSE_FB_W` VEL_K).
    pub fn run_vel_track(
        &mut self,
        cmd: ManeuverCommand,
        frame: VelocityFrame,
        target: Vector3f,
        max_vel_linear: f32,
        max_vel_angular: f32,
        max_accel_linear: f32,
        max_accel_angular: f32,
    ) -> Result<ManeuverSetpoints, ControlsError> {
        let mut target = target;
        let v = hypotf(target.x, target.y);
        if v > max_vel_linear {
            target.x *= max_vel_linear / v;
            target.y *= max_vel_linear / v;
        }
        target.z = target.z.clamp(-max_vel_angular, max_vel_angular);

        // No capability cap here (unlike position tracking): without position
        // feedback nothing chases a ramp the robot can't follow, and the output
        // stage's friction-first saturation keeps the acceleration direction, so the
        // robot simply accelerates as hard as the wheels allow.
        let heading = self.state_estimate.z;
        let dt = self.dt;
        let accel_linear = max_accel_linear;
        let make = |seed: Vector6f| {
            Ok(TrackedTrajectory::VelocityRamp(VelocityRamp::new(
                seed,
                heading,
                frame,
                target,
                accel_linear,
                max_accel_angular,
                dt,
            )))
        };
        let mut setpoints = self.run_traj_open_loop(cmd, make)?;
        let k_vel_w = self.pose_fb_w[3];
        setpoints.body_twist.z += (setpoints.body_twist.z - self.yaw_rate_fb) * k_vel_w;
        Ok(setpoints)
    }

    fn xy_feedback_saturated(&self) -> bool {
        let Some(traj) = self.trajectory.as_ref() else {
            return false;
        };
        let traj_pos = traj.sample().0;
        hypotf(traj_pos.x - self.state_estimate.x, traj_pos.y - self.state_estimate.y)
            > TRAJ_RESET_ERROR_XY
    }

    /// Replan when there is no trajectory or `cmd` changed. The seed is the
    /// current trajectory state when `seed_from_traj` and a trajectory exists,
    /// otherwise the state estimate.
    fn plan_if_needed<F>(
        &mut self,
        cmd: ManeuverCommand,
        seed_from_traj: bool,
        make_traj: F,
    ) -> Result<(), ControlsError>
    where
        F: FnOnce(Vector6f) -> Result<TrackedTrajectory, ControlsError>,
    {
        if self.trajectory.is_none() || self.command_changed(&cmd) {
            let seed = match self.trajectory.as_ref() {
                Some(traj) if seed_from_traj => traj.sample().0,
                _ => self.state_estimate,
            };
            match make_traj(seed) {
                Ok(new_traj) => {
                    self.trajectory = Some(new_traj);
                }
                Err(e) => {
                    self.trajectory = None;
                    self.prev_cmd = None;
                    return Err(e);
                }
            }
        }
        self.prev_cmd = Some(cmd);
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Largest linear acceleration (m/s^2) the output stage can deliver along the
    /// local direction `dir_local` while already moving that way: the same
    /// friction-first fit as `BodyController` (full Coulomb compensation, then the
    /// largest acceleration that keeps every wheel within its thermally derated
    /// feed-forward current limit). Strongly direction dependent on an omni robot
    /// (robot 2: ~2.6 m/s^2 forward vs ~1.1 m/s^2 sideways at 3.5 A).
    pub fn feasible_linear_accel(&self, dir_local: SVector<f32, 2>) -> f32 {
        let n = dir_local.norm();
        if n < 1e-6 {
            return f32::INFINITY;
        }
        let u = Vector3f::new(dir_local.x / n, dir_local.y / n, 0.0);
        let m = &self.robot_model;
        let to_currents = |a: Vector3f| m.torques_to_currents(m.transform_accel2wheel_local() * a);
        let i_fric = to_currents(m.feedforward_local_accel(u * 0.5, Vector3f::zeros()));
        let i_unit = to_currents(m.feedforward_local_accel(Vector3f::zeros(), u));
        let limits = self.wheel_ff_current_limits;
        let mut s = f32::INFINITY;
        for i in 0..4 {
            if fabsf(i_unit[i]) < 1e-6 {
                continue;
            }
            let headroom = if i_unit[i] > 0.0 {
                limits[i] - i_fric[i]
            } else {
                limits[i] + i_fric[i]
            };
            s = s.min(headroom / fabsf(i_unit[i]));
        }
        s.max(0.0)
    }

    /// Feasible linear acceleration for a straight move by `delta_xy` (global) while
    /// the heading turns from `heading_from` to `heading_to`: the minimum of
    /// `feasible_linear_accel` over headings sampled along the turn.
    pub fn feasible_linear_accel_for_move(
        &self,
        delta_xy: SVector<f32, 2>,
        heading_from: f32,
        heading_to: f32,
    ) -> f32 {
        const SAMPLES: usize = 5;
        let turn = remainderf(heading_to - heading_from, 2.0 * PI);
        let mut a = f32::INFINITY;
        for k in 0..SAMPLES {
            let h = heading_from + turn * (k as f32) / ((SAMPLES - 1) as f32);
            let (sn, cs) = (libm::sinf(h), libm::cosf(h));
            let local = SVector::<f32, 2>::new(
                cs * delta_xy.x + sn * delta_xy.y,
                -sn * delta_xy.x + cs * delta_xy.y,
            );
            a = a.min(self.feasible_linear_accel(local));
        }
        a
    }

    /// PID+FF trajectory tracking: compute body-frame setpoints from the
    /// trajectory's current position/velocity/acceleration sample.
    fn calc_tracking_setpoints(
        &mut self,
        traj_state: Vector6f,
        traj_accel: Vector3f,
    ) -> Result<ManeuverSetpoints, ControlsError> {
        // Trajectory tracking after TIGERs Mannheim `controlXY` / `controlW`
        // (Firmware src/robot/ctrl_panthera.c), in the global frame. Position
        // error feeds the velocity setpoint; the acceleration output is the
        // trajectory feed-forward only. Both fade to zero inside the
        // anti-jitter thresholds so the outputs are exactly zero at rest.
        let state_estimate = self.state_estimate;
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let traj_pos: Vector3f = traj_state.fixed_rows::<3>(0).into();
        let traj_vel: Vector3f = traj_state.fixed_rows::<3>(3).into();

        // XY: P on position, clamped in magnitude, then anti-jitter scaled.
        let [k_xy, max_xy, aj_xy] = [self.pose_fb_xy[0], self.pose_fb_xy[1], self.pose_fb_xy[2]];
        let err_x = traj_pos.x - pose_estimate.x;
        let err_y = traj_pos.y - pose_estimate.y;
        let err_xy = hypotf(err_x, err_y);
        let mut fb_x = err_x * k_xy;
        let mut fb_y = err_y * k_xy;
        let fb_xy = hypotf(fb_x, fb_y);
        if fb_xy > max_xy && fb_xy > 0.0 {
            fb_x *= max_xy / fb_xy;
            fb_y *= max_xy / fb_xy;
        }
        let scale_xy = anti_jitter_scale(err_xy, aj_xy);

        // W: P on orientation (clamped) plus P on angular velocity, anti-jitter
        // scaled by the orientation error.
        let [k_w, max_w, aj_w, k_vel_w] =
            [self.pose_fb_w[0], self.pose_fb_w[1], self.pose_fb_w[2], self.pose_fb_w[3]];
        let err_w = remainderf(traj_pos.z - pose_estimate.z, 2.0 * PI);
        let pos_fb_w = (err_w * k_w).clamp(-max_w, max_w);
        let vel_fb_w = (traj_vel.z - self.yaw_rate_fb) * k_vel_w;
        let scale_w = anti_jitter_scale(fabsf(err_w), aj_w);

        let twist_out = Vector3f::new(
            traj_vel.x + fb_x * scale_xy,
            traj_vel.y + fb_y * scale_xy,
            traj_vel.z + (pos_fb_w + vel_fb_w) * scale_w,
        );
        let accel_out = Vector3f::new(
            traj_accel.x * scale_xy,
            traj_accel.y * scale_xy,
            traj_accel.z * scale_w,
        );

        Ok(ManeuverSetpoints {
            body_twist: twist_out,
            body_accel: accel_out,
        })
    }

    fn tracking_error_exceeded(&self) -> bool {
        let traj_state = match self.trajectory.as_ref() {
            Some(t) => t.sample().0,
            None => return false,
        };
        let traj_state_pose: Vector3f = traj_state.fixed_rows::<3>(0).into();
        let traj_state_twist: Vector3f = traj_state.fixed_rows::<3>(3).into();
        let pose_estimate: Vector3f = self.state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = self.state_estimate.fixed_rows::<3>(3).into();
        let linear_pose_error = hypotf(
            traj_state_pose.x - pose_estimate.x,
            traj_state_pose.y - pose_estimate.y,
        );
        let angular_pose_error = fabsf(remainderf(traj_state_pose.z - pose_estimate.z, 2.0 * PI));
        let linear_twist_error = hypotf(
            traj_state_twist.x - twist_estimate.x,
            traj_state_twist.y - twist_estimate.y,
        );
        let angular_twist_error = fabsf(traj_state_twist.z - twist_estimate.z);
        linear_pose_error > self.tracking_divergence_thresh[0]
            || angular_pose_error > self.tracking_divergence_thresh[1]
            || linear_twist_error > self.tracking_divergence_thresh[2]
            || angular_twist_error > self.tracking_divergence_thresh[3]
    }

    // -----------------------------------------------------------------------
    // Parameter read/write (data operations only — reset is caller's concern)
    // -----------------------------------------------------------------------

    pub fn expected_format(name: ParameterName::Type) -> Option<ParameterDataFormat::Type> {
        match name {
            ParameterName::KF_PROCESS_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MEASUREMENT_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_WHEEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_INERTIA => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_MOTOR_MODEL => Some(ParameterDataFormat::F32),
            ParameterName::PHYS_FRICTION_MODEL => Some(ParameterDataFormat::VEC6_F32),
            ParameterName::TRACKING_DIVERGENCE_THRESHOLD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_XY => Some(ParameterDataFormat::VEC3_F32),
            ParameterName::POSE_FB_W => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::WHEEL_VEL_FB => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::WHEEL_FF_CURRENT_LIMIT => Some(ParameterDataFormat::F32),
            ParameterName::WHEEL_SLIP_COMP => Some(ParameterDataFormat::F32),
            ParameterName::VISION_SEED_SAMPLES => Some(ParameterDataFormat::F32),
            _ => None,
        }
    }

    pub fn read_param(&self, name: ParameterName::Type, reply: &mut ParameterCommand) {
        let phys = self.robot_model.get_params();
        match name {
            ParameterName::KF_PROCESS_STD => {
                // EKF process covariance Q -> std (sqrt of diagonal variances).
                // Q diag = [pos_lin, pos_lin, pos_ang, vel_lin, vel_lin]; the
                // KF_PROCESS_STD vel-angular slot has no EKF state and reads 0.
                let q = self.state_estimator.ekf.get_params().q;
                reply.data.vec4_f32 = [
                    sqrtf(q[(0, 0)]),
                    sqrtf(q[(2, 2)]),
                    sqrtf(q[(3, 3)]),
                    0.0,
                ];
            }
            ParameterName::KF_MEASUREMENT_STD => {
                // EKF measurement covariance R -> std (sqrt of diagonal
                // variances). R diag = [vision_lin, vision_lin, vision_ang];
                // the encoder/gyro slots are unused by the EKF and read 0.
                let r = self.state_estimator.ekf.get_params().r;
                reply.data.vec4_f32 = [
                    sqrtf(r[(0, 0)]),
                    sqrtf(r[(2, 2)]),
                    0.0,
                    0.0,
                ];
            }
            ParameterName::PHYS_WHEEL => {
                reply.data.vec4_f32 = [phys.alpha_rad, phys.beta_rad, phys.l_m, phys.r_m];
            }
            ParameterName::PHYS_INERTIA => {
                reply.data.vec2_f32 = [phys.mass_kg, phys.iz_kgm2];
            }
            ParameterName::PHYS_MOTOR_MODEL => {
                reply.data.f32_ = phys.motor_torque_constant_nmpa;
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                reply.data.vec6_f32 = [
                    phys.coulomb_xy_n,
                    phys.viscous_xy_nspm,
                    phys.coulomb_w_nm,
                    phys.viscous_w_nmsprad,
                    phys.efficiency_xy,
                    phys.efficiency_w,
                ];
            }
            ParameterName::TRACKING_DIVERGENCE_THRESHOLD => {
                reply.data.vec4_f32 = self.tracking_divergence_thresh.into();
            }
            ParameterName::VISION_SEED_SAMPLES => {
                reply.data.f32_ = self.state_estimator.vision_filter.get_params().seed_samples as f32;
            }
            ParameterName::POSE_FB_XY => {
                reply.data.vec3_f32 = self.pose_fb_xy.into();
            }
            ParameterName::POSE_FB_W => {
                reply.data.vec4_f32 = self.pose_fb_w.into();
            }
            ParameterName::WHEEL_VEL_FB => {
                reply.data.vec4_f32 = self.wheel_vel_fb.into();
            }
            ParameterName::WHEEL_FF_CURRENT_LIMIT => {
                reply.data.f32_ = self.wheel_ff_current_limit;
            }
            ParameterName::WHEEL_SLIP_COMP => {
                reply.data.f32_ = self.wheel_slip_comp;
            }
            _ => unreachable!(),
        }
    }

    pub fn write_param(&mut self, cmd: &ParameterCommand) {
        match cmd.parameter_name {
            ParameterName::KF_PROCESS_STD => {
                // std -> variance (square); build EKF process covariance Q.
                // Q diag = [pos_lin, pos_lin, pos_ang, vel_lin, vel_lin]; the
                // vel-angular std slot (v[3]) has no EKF state and is ignored.
                let v = unsafe { cmd.data.vec4_f32 };
                let pos_lin_var = v[0] * v[0];
                let pos_ang_var = v[1] * v[1];
                let vel_lin_var = v[2] * v[2];

                let vision_filter_params = self.state_estimator.vision_filter.get_params();
                let mut ekf_params = self.state_estimator.ekf.get_params();
                let odometer_params = self.state_estimator.odometer.get_params();
                ekf_params.q = SMatrix::<f32, EKF_STATE_LEN, EKF_STATE_LEN>::from_diagonal(
                    &SVector::<f32, EKF_STATE_LEN>::from([
                        pos_lin_var,
                        pos_lin_var,
                        pos_ang_var,
                        vel_lin_var,
                        vel_lin_var,
                    ])
                );
                let ekf = BufferedEKF::new(
                    ekf_params,
                    SVector::<f32, 3>::zeros(),
                    SVector::<f32, 3>::zeros(),
                );
                let vision_filter = VisionFilter::new(vision_filter_params);
                let odometer = Odometer::new(odometer_params);
                self.state_estimator = StateEstimator::new(
                    ekf,
                    vision_filter,
                    odometer,
                );
                self.reset();
            }
            ParameterName::KF_MEASUREMENT_STD => {
                // std -> variance (square); build EKF measurement covariance R.
                // R diag = [vision_lin, vision_lin, vision_ang]; the encoder
                // (v[2]) and gyro (v[3]) std slots are unused by the EKF.
                let v = unsafe { cmd.data.vec4_f32 };
                let vision_lin_var = v[0] * v[0];
                let vision_ang_var = v[1] * v[1];

                let vision_filter_params = self.state_estimator.vision_filter.get_params();
                let mut ekf_params = self.state_estimator.ekf.get_params();
                let odometer_params = self.state_estimator.odometer.get_params();
                ekf_params.r = SMatrix::<f32, EKF_MEAS_LEN, EKF_MEAS_LEN>::from_diagonal(
                    &SVector::<f32, EKF_MEAS_LEN>::from([
                        vision_lin_var,
                        vision_lin_var,
                        vision_ang_var,
                    ]),
                );
                let ekf = BufferedEKF::new(
                    ekf_params,
                    SVector::<f32, 3>::zeros(),
                    SVector::<f32, 3>::zeros(),
                );
                let vision_filter = VisionFilter::new(vision_filter_params);
                let odometer = Odometer::new(odometer_params);
                self.state_estimator = StateEstimator::new(
                    ekf,
                    vision_filter,
                    odometer,
                );
                self.reset();
            }
            ParameterName::PHYS_WHEEL => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut p = self.robot_model.get_params();
                p.alpha_rad = v[0];
                p.beta_rad = v[1];
                p.l_m = v[2];
                p.r_m = v[3];
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::PHYS_INERTIA => {
                let v = unsafe { cmd.data.vec2_f32 };
                let mut p = self.robot_model.get_params();
                p.mass_kg = v[0];
                p.iz_kgm2 = v[1];
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::PHYS_MOTOR_MODEL => {
                let v = unsafe { cmd.data.f32_ };
                let mut p = self.robot_model.get_params();
                p.motor_torque_constant_nmpa = v;
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                let v = unsafe { cmd.data.vec6_f32 };
                let mut p = self.robot_model.get_params();
                p.coulomb_xy_n = v[0];
                p.viscous_xy_nspm = v[1];
                p.coulomb_w_nm = v[2];
                p.viscous_w_nmsprad = v[3];
                p.efficiency_xy = v[4];
                p.efficiency_w = v[5];
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::TRACKING_DIVERGENCE_THRESHOLD => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.tracking_divergence_thresh = Vector4f::new(v[0], v[1], v[2], v[3]);
            }
            ParameterName::VISION_SEED_SAMPLES => {
                let v = unsafe { cmd.data.f32_ };
                let mut params = self.state_estimator.vision_filter.get_params();
                params.seed_samples = if v.is_finite() {
                    (libm::roundf(v) as usize).clamp(1, DEFAULT_VISION_BUFF_LEN)
                } else {
                    params.seed_samples
                };
                // Takes effect on the next vision reseed (apply_command resets afterwards).
                self.state_estimator.vision_filter = VisionFilter::new(params);
            }
            ParameterName::POSE_FB_XY => {
                let v = unsafe { cmd.data.vec3_f32 };
                if v.iter().all(|x| x.is_finite() && *x >= 0.0) {
                    self.pose_fb_xy = Vector3f::new(v[0], v[1], v[2]);
                }
            }
            ParameterName::POSE_FB_W => {
                let v = unsafe { cmd.data.vec4_f32 };
                if v.iter().all(|x| x.is_finite() && *x >= 0.0) {
                    self.pose_fb_w = Vector4f::new(v[0], v[1], v[2], v[3]);
                }
            }
            ParameterName::WHEEL_VEL_FB => {
                let v = unsafe { cmd.data.vec4_f32 };
                if v.iter().all(|x| x.is_finite() && *x >= 0.0) {
                    self.wheel_vel_fb = Vector4f::new(v[0], v[1], v[2], v[3].min(6.3));
                }
            }
            ParameterName::WHEEL_FF_CURRENT_LIMIT => {
                let v = unsafe { cmd.data.f32_ };
                if v.is_finite() && v >= 0.0 {
                    self.wheel_ff_current_limit = v;
                }
            }
            ParameterName::WHEEL_SLIP_COMP => {
                let v = unsafe { cmd.data.f32_ };
                if v.is_finite() && (0.0..=3.0).contains(&v) {
                    self.wheel_slip_comp = v;
                }
            }
            _ => unreachable!(),
        }
    }
}

/// TIGERs `MapToRangef32(0, thresh, 0, 1, err)`: linear 0..1 ramp over
/// [0, thresh], 1 above it (and always 1 when the threshold is disabled).
fn anti_jitter_scale(err_abs: f32, thresh: f32) -> f32 {
    if thresh <= 0.0 {
        1.0
    } else {
        (err_abs / thresh).clamp(0.0, 1.0)
    }
}
