use crate::motion::params::controller_params::{
    ENC_LAG_K, ENC_LAG_MODE, ENC_LAG_T_HORIZON, ENC_LAG_T_SLOPE,
    FRICTION_COMP_HOLD_AWAY_SPEED, FRICTION_COMP_HOLD_RELEASE_FRAC,
    TRACKING_DIVERGENCE_RECOVERY_REST_TICKS,
    TRACKING_DIVERGENCE_RECOVERY_REST_WHEEL_VEL,
};
use crate::motion::accel_trajectory::AccelTrajectory;
use crate::motion::pid::PidController;
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
    z_rotation_mat, ControlsError, Vector2f, Vector3f, Vector4f, Vector6f, Vector8f,
};
use ateam_lib_stm32::model::{FirstOrderLag, FirstOrderLagParams};
use core::f32::consts::PI;
use embassy_time::{Duration, Instant};
use libm::{fabsf, hypotf, remainderf, sqrtf};
use nalgebra::{SMatrix, SVector};

pub(crate) const VISION_ACTIVE_TIMEOUT_S: f32 = 0.5;

/// All trajectory types the firmware can track.
///
/// Provides static dispatch over trajectory types that each implement the
/// `Trajectory` trait directly (owning their own state).
pub enum TrackedTrajectory {
    BangBang(BangBangTraj3D),
    Pivot(PivotTrajectory),
    Linear(LinearTrajectory),
    Accel(AccelTrajectory),
}

impl Trajectory for TrackedTrajectory {
    fn tick(&mut self, dt: f32) {
        match self {
            Self::BangBang(t) => t.tick(dt),
            Self::Pivot(t) => t.tick(dt),
            Self::Linear(t) => t.tick(dt),
            Self::Accel(t) => t.tick(dt),
        }
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        match self {
            Self::BangBang(t) => t.sample(),
            Self::Pivot(t) => t.sample(),
            Self::Linear(t) => t.sample(),
            Self::Accel(t) => t.sample(),
        }
    }
}

impl TrackedTrajectory {
    /// Like `sample`, but bang-bang accel is averaged over the coming tick so
    /// switches inside the tick don't produce `+a_max`/`-a_max` chatter.
    fn sample_over(&self, dt: f32) -> (Vector6f, Vector3f) {
        match self {
            Self::BangBang(t) => {
                let (state, accel) = t.sample();
                (state, t.mean_accel(dt).unwrap_or(accel))
            }
            _ => self.sample(),
        }
    }

    /// Trajectory velocity `lookahead` seconds ahead of now. Bang-bang is
    /// evaluated exactly; other trajectories extrapolate with the current accel.
    fn vel_ahead(&self, lookahead: f32) -> Vector3f {
        let (state, accel) = self.sample();
        let vel: Vector3f = state.fixed_rows::<3>(3).into();
        if lookahead <= 0.0 {
            return vel;
        }
        match self {
            Self::BangBang(t) => t
                .state_at(lookahead)
                .map(|s| s.fixed_rows::<3>(3).into())
                .unwrap_or(vel + accel * lookahead),
            _ => vel + accel * lookahead,
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
    pub pose_pid_controller: PidController<3>,
    /// Tracking accel (torque) path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub tracking_accel_gain: Vector2f,
    /// Feedforward accel low-pass cutoffs: [CUTOFF_LINEAR_HZ, CUTOFF_ANGULAR_HZ]; <= 0 disables
    pub tracking_ff_accel_cutoff_hz: Vector2f,
    /// Lookahead (s) for the trajectory velocity used as the body velocity output
    pub tracking_vel_lookahead_s: f32,
    /// [HOLD_VEL_LOOP_OFF, KV_LINEAR, KV_ANGULAR] wheel velocity setpoint feedback options
    pub tracking_wheel_vel_fb: Vector3f,
    /// Set this tick when the wheel velocity loop should be off (closed-loop
    /// trajectory holding with TRACKING_WHEEL_VEL_FB[0] set). Cleared every tick.
    pub wheel_vel_loop_suppressed: bool,
    /// Low-passed feedforward accel; `None` until the next tracking tick seeds it.
    pub tracking_ff_accel_filt: Option<Vector3f>,
    /// Divergence thresholds: [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
    pub tracking_divergence_thresh: Vector4f,
    /// [LINEAR_VEL_THRESHOLD, ANGULAR_VEL_THRESHOLD]
    pub friction_comp_vel_thresh: Vector2f,
    /// [ERROR_LINEAR, ERROR_ANGULAR] hold breakaway friction comp error thresholds; <= 0 disables
    pub friction_comp_hold_err_thresh: Vector2f,
    /// Hold breakaway comp latched on per group [linear, angular] (hysteresis).
    pub friction_comp_hold_active: [bool; 2],
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
    /// holds it until the wheels stop, then resets the controller.
    pub tracking_divergence_state: TrackingDivergenceState,
    /// Consecutive ticks the wheels have been near rest while recovering.
    pub tracking_recovery_at_rest_ticks: u32,
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
            pose_pid_controller: PidController::<3>::from_gains_matrix_with_anti_jitter(
                &controller_params::pose_pid_gains(),
                Some(controller_params::POSE_PID_ANTI_JITTER_THRESH),
            ),
            tracking_accel_gain: controller_params::TRACKING_ACCEL_GAIN,
            tracking_ff_accel_cutoff_hz: controller_params::TRACKING_FF_ACCEL_CUTOFF_HZ,
            tracking_vel_lookahead_s: controller_params::TRACKING_VEL_LOOKAHEAD_S,
            tracking_wheel_vel_fb: controller_params::TRACKING_WHEEL_VEL_FB,
            wheel_vel_loop_suppressed: false,
            tracking_ff_accel_filt: None,
            tracking_divergence_thresh: controller_params::TRACKING_DIVERGENCE_THRESHOLD,
            friction_comp_vel_thresh: controller_params::FRICTION_COMP_VEL_THRESHOLD,
            friction_comp_hold_err_thresh: controller_params::FRICTION_COMP_HOLD_ERR_THRESHOLD,
            friction_comp_hold_active: [false; 2],
            trajectory: None,
            prev_cmd: None,
            trajectory_open_loop: false,
            enc_lag,
            dt,
            state_estimate: Vector6f::zeros(),
            state_estimate_vel_local: Vector3f::zeros(),
            tracking_divergence_state: TrackingDivergenceState::Normal,
            tracking_recovery_at_rest_ticks: 0,
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
        self.pose_pid_controller.reset();
        self.trajectory = None;
        self.prev_cmd = None;
        self.trajectory_open_loop = false;
        self.tracking_ff_accel_filt = None;
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
        self.tracking_ff_accel_filt = None;
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

        Ok(state_prediction)
    }

    /// Feed-forward friction force (global frame) for the commanded body twist.
    ///
    /// Both the Coulomb sign and the viscous magnitude come from the commanded
    /// (target) twist, not the state estimate. Linear compensation is on while the
    /// commanded local XY speed is at least FRICTION_COMP_VEL_THRESHOLD[0]; angular
    /// while the commanded angular speed is at least FRICTION_COMP_VEL_THRESHOLD[1].
    ///
    /// When an axis group is off and a closed-loop trajectory is held, breakaway
    /// friction comp is applied toward the target instead. It latches on when the
    /// pose error exceeds FRICTION_COMP_HOLD_ERR_THRESHOLD and stays on until the
    /// error drops below FRICTION_COMP_HOLD_RELEASE_FRAC of it, so it doesn't
    /// toggle with the robot's springback around the threshold. It is off while the
    /// robot moves away from the target faster than FRICTION_COMP_HOLD_AWAY_SPEED.
    pub fn compute_friction(&mut self, body_twist: Vector3f) -> Vector3f {
        // Tiny speed so the model returns its zero-speed (breakaway) magnitude.
        const BREAKAWAY_SPEED: f32 = 1e-6;

        let theta = self.state_estimate.z;
        let cmd_twist_local: Vector3f = z_rotation_mat(-theta) * body_twist;

        let linear_comp_on =
            hypotf(cmd_twist_local.x, cmd_twist_local.y) >= self.friction_comp_vel_thresh[0];
        let angular_comp_on = fabsf(cmd_twist_local.z) >= self.friction_comp_vel_thresh[1];

        let mut fric_twist_local = Vector3f::new(
            if linear_comp_on { cmd_twist_local.x } else { 0.0 },
            if linear_comp_on { cmd_twist_local.y } else { 0.0 },
            if angular_comp_on { cmd_twist_local.z } else { 0.0 },
        );

        let held_traj = match self.trajectory.as_ref() {
            Some(t) if !self.trajectory_open_loop => Some(t.sample().0),
            _ => None,
        };
        let mut hold_active = [false; 2];
        if let Some(traj_state) = held_traj {
            let est_vel: Vector3f = self.state_estimate.fixed_rows::<3>(3).into();
            let err_xy = Vector2f::new(
                traj_state[0] - self.state_estimate[0],
                traj_state[1] - self.state_estimate[1],
            );
            let err_th = remainderf(traj_state[2] - theta, 2.0 * PI);

            // Latch on above the threshold, release below RELEASE_FRAC of it.
            let latch = |was_active: bool, err: f32, thresh: f32| {
                thresh > 0.0
                    && if was_active {
                        err > FRICTION_COMP_HOLD_RELEASE_FRAC * thresh
                    } else {
                        err > thresh
                    }
            };

            let err_xy_norm = err_xy.norm();
            if !linear_comp_on
                && latch(self.friction_comp_hold_active[0], err_xy_norm, self.friction_comp_hold_err_thresh[0])
            {
                hold_active[0] = true;
                if err_xy.dot(&est_vel.xy()) / err_xy_norm > -FRICTION_COMP_HOLD_AWAY_SPEED[0] {
                    let dir_local =
                        z_rotation_mat(-theta) * Vector3f::new(err_xy.x, err_xy.y, 0.0) / err_xy_norm;
                    fric_twist_local.x = BREAKAWAY_SPEED * dir_local.x;
                    fric_twist_local.y = BREAKAWAY_SPEED * dir_local.y;
                }
            }

            if !angular_comp_on
                && latch(self.friction_comp_hold_active[1], fabsf(err_th), self.friction_comp_hold_err_thresh[1])
            {
                hold_active[1] = true;
                if err_th.signum() * est_vel.z > -FRICTION_COMP_HOLD_AWAY_SPEED[1] {
                    fric_twist_local.z = BREAKAWAY_SPEED * err_th.signum();
                }
            }
        }
        self.friction_comp_hold_active = hold_active;

        let friction_force_local = self.robot_model.compute_local_friction_force(fric_twist_local);
        z_rotation_mat(theta) * friction_force_local
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
    ///   controller is reset (replanning from the fresh state estimate) and
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
                    self.reset(); // returns to Normal, clears trajectory
                    defmt::warn!("recovery complete, controller reset");
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
        let seed_from_traj = !self.trajectory_open_loop;
        self.plan_if_needed(cmd, seed_from_traj, make_traj)?;
        self.trajectory_open_loop = false;

        let traj = self
            .trajectory
            .as_ref()
            .ok_or(ControlsError::InvalidInput)?;
        let (traj_state, traj_accel) = traj.sample_over(self.dt);
        let traj_vel_ahead = traj.vel_ahead(self.tracking_vel_lookahead_s);
        // A held trajectory commands 0 wheel velocity, so the wheel velocity loop
        // would resist the pose feedback that is still correcting the error.
        const HOLD_VEL_EPS: f32 = 1e-4;
        self.wheel_vel_loop_suppressed = self.tracking_wheel_vel_fb[0] != 0.0
            && traj_vel_ahead.xy().norm() < HOLD_VEL_EPS
            && fabsf(traj_vel_ahead.z) < HOLD_VEL_EPS;

        let setpoints = self.calc_tracking_setpoints(traj_state, traj_accel, traj_vel_ahead)?;

        // Advance the trajectory's internal clock by dt.
        // After this, traj.sample() returns the state at the start of the next tick.
        if let Some(traj) = &mut self.trajectory {
            traj.tick(self.dt);
        }

        Ok(setpoints)
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
        self.tracking_ff_accel_filt = None;

        let heading = self.state_estimate.z;
        let traj = self
            .trajectory
            .as_mut()
            .ok_or(ControlsError::InvalidInput)?;
        if let TrackedTrajectory::Accel(t) = traj {
            t.set_body_heading(heading);
        }
        let (traj_state, traj_accel) = traj.sample_over(self.dt);
        traj.tick(self.dt);

        Ok(ManeuverSetpoints {
            body_twist: traj_state.fixed_rows::<3>(3).into(),
            body_accel: traj_accel,
        })
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

    /// PID+FF trajectory tracking: compute body-frame setpoints from the
    /// trajectory's current position/velocity/acceleration sample.
    /// `traj_vel_ahead` (trajectory velocity at TRACKING_VEL_LOOKAHEAD_S) is the
    /// body velocity output; the PID derivative uses the current `traj_state`.
    fn calc_tracking_setpoints(
        &mut self,
        traj_state: Vector6f,
        traj_accel: Vector3f,
        traj_vel_ahead: Vector3f,
    ) -> Result<ManeuverSetpoints, ControlsError> {
        let state_estimate = self.state_estimate;
        let pose_estimate: Vector3f = state_estimate.fixed_rows::<3>(0).into();
        let twist_estimate: Vector3f = state_estimate.fixed_rows::<3>(3).into();

        let mut traj_pos: Vector3f = traj_state.fixed_rows::<3>(0).into();
        let traj_vel: Vector3f = traj_state.fixed_rows::<3>(3).into();

        // Wrap θ to avoid PID over-rotation at the ±π discontinuity.
        traj_pos.z = pose_estimate.z + remainderf(traj_pos.z - pose_estimate.z, 2.0 * PI);

        let twist_error = traj_vel - twist_estimate;
        let pos_pid_feedback = self.pose_pid_controller.calculate_with_derivative(
            &traj_pos,
            &pose_estimate,
            &twist_error,
            self.dt,
        );

        // Wheel torque path: gain-weighted (low-passed) trajectory accel FF + PID FB.
        let ff_accel = self.filter_ff_accel(traj_accel);
        let accel_out: Vector3f =
            self.tracking_accel_gain[0] * ff_accel + self.tracking_accel_gain[1] * pos_pid_feedback;

        // Wheel velocity path: the trajectory velocity looked ahead to cover the
        // coming tick and command latency (also the friction comp input), plus
        // optional proportional pose feedback so the wheel velocity loop drives
        // toward the target instead of holding the wheels at the trajectory speed.
        let pos_error = traj_pos - pose_estimate;
        let kv = Vector3f::new(
            self.tracking_wheel_vel_fb[1],
            self.tracking_wheel_vel_fb[1],
            self.tracking_wheel_vel_fb[2],
        );
        let twist_out: Vector3f = traj_vel_ahead + kv.component_mul(&pos_error);

        Ok(ManeuverSetpoints {
            body_twist: twist_out,
            body_accel: accel_out,
        })
    }

    /// First-order low-pass on the feedforward accel, linear cutoff on x/y and
    /// angular cutoff on θ. Seeds from the raw value after a reset.
    fn filter_ff_accel(&mut self, raw: Vector3f) -> Vector3f {
        let alpha = |hz: f32| {
            if hz > 0.0 {
                1.0 - libm::expf(-2.0 * PI * hz * self.dt)
            } else {
                1.0
            }
        };
        let a_lin = alpha(self.tracking_ff_accel_cutoff_hz[0]);
        let a_ang = alpha(self.tracking_ff_accel_cutoff_hz[1]);
        let filt = match self.tracking_ff_accel_filt {
            Some(prev) => Vector3f::new(
                prev.x + a_lin * (raw.x - prev.x),
                prev.y + a_lin * (raw.y - prev.y),
                prev.z + a_ang * (raw.z - prev.z),
            ),
            None => raw,
        };
        self.tracking_ff_accel_filt = Some(filt);
        filt
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
            ParameterName::PHYS_MOTOR_MODEL => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_FRICTION_MODEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_STRIBECK_MODEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::FRICTION_COMP_VEL_THRESHOLD => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRACKING_ACCEL_GAIN => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRACKING_FF_ACCEL_CUTOFF_HZ => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRACKING_VEL_LOOKAHEAD_S => Some(ParameterDataFormat::F32),
            ParameterName::TRACKING_WHEEL_VEL_FB => Some(ParameterDataFormat::VEC3_F32),
            ParameterName::FRICTION_COMP_HOLD_ERR_THRESHOLD => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRACKING_DIVERGENCE_THRESHOLD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_PIDII_LINEAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_ANGULAR => Some(ParameterDataFormat::VEC5_F32),
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
                reply.data.vec2_f32 = [phys.motor_torque_constant_nmpa, phys.motor_efficiency_factor];
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                reply.data.vec4_f32 = [
                    phys.coulomb_friction_linear_n,
                    phys.coulomb_friction_angular_nm,
                    phys.viscous_friction_linear_nspm,
                    phys.viscous_friction_angular_nmsprad,
                ];
            }
            ParameterName::PHYS_STRIBECK_MODEL => {
                reply.data.vec4_f32 = [
                    phys.static_friction_linear_n,
                    phys.static_friction_angular_nm,
                    phys.stribeck_vel_linear_mps,
                    phys.stribeck_vel_angular_radps,
                ];
            }
            ParameterName::FRICTION_COMP_VEL_THRESHOLD => {
                reply.data.vec2_f32 = self.friction_comp_vel_thresh.into();
            }
            ParameterName::FRICTION_COMP_HOLD_ERR_THRESHOLD => {
                reply.data.vec2_f32 = self.friction_comp_hold_err_thresh.into();
            }
            ParameterName::TRACKING_ACCEL_GAIN => {
                reply.data.vec2_f32 = self.tracking_accel_gain.into();
            }
            ParameterName::TRACKING_FF_ACCEL_CUTOFF_HZ => {
                reply.data.vec2_f32 = self.tracking_ff_accel_cutoff_hz.into();
            }
            ParameterName::TRACKING_VEL_LOOKAHEAD_S => {
                reply.data.f32_ = self.tracking_vel_lookahead_s;
            }
            ParameterName::TRACKING_WHEEL_VEL_FB => {
                reply.data.vec3_f32 = self.tracking_wheel_vel_fb.into();
            }
            ParameterName::TRACKING_DIVERGENCE_THRESHOLD => {
                reply.data.vec4_f32 = self.tracking_divergence_thresh.into();
            }
            ParameterName::VISION_SEED_SAMPLES => {
                reply.data.f32_ = self.state_estimator.vision_filter.get_params().seed_samples as f32;
            }
            ParameterName::POSE_FB_PIDII_LINEAR | ParameterName::POSE_FB_PIDII_ANGULAR => {
                let gain = self.pose_pid_controller.get_gain();
                let row = match name {
                    ParameterName::POSE_FB_PIDII_LINEAR => 0,
                    _ => 2,
                };
                reply.data.vec5_f32 = [
                    gain[(row, 0)],
                    gain[(row, 1)],
                    gain[(row, 2)],
                    gain[(row, 3)],
                    gain[(row, 4)],
                ];
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
                let v = unsafe { cmd.data.vec2_f32 };
                let mut p = self.robot_model.get_params();
                p.motor_torque_constant_nmpa = v[0];
                p.motor_efficiency_factor = v[1];
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut p = self.robot_model.get_params();
                p.coulomb_friction_linear_n = v[0];
                p.coulomb_friction_angular_nm = v[1];
                p.viscous_friction_linear_nspm = v[2];
                p.viscous_friction_angular_nmsprad = v[3];
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::PHYS_STRIBECK_MODEL => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut p = self.robot_model.get_params();
                p.static_friction_linear_n = v[0];
                p.static_friction_angular_nm = v[1];
                p.stribeck_vel_linear_mps = v[2];
                p.stribeck_vel_angular_radps = v[3];
                let _ = self.robot_model.update_params(p);
            }
            ParameterName::FRICTION_COMP_VEL_THRESHOLD => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.friction_comp_vel_thresh = Vector2f::new(v[0], v[1]);
            }
            ParameterName::FRICTION_COMP_HOLD_ERR_THRESHOLD => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.friction_comp_hold_err_thresh = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRACKING_ACCEL_GAIN => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.tracking_accel_gain = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRACKING_FF_ACCEL_CUTOFF_HZ => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.tracking_ff_accel_cutoff_hz = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRACKING_VEL_LOOKAHEAD_S => {
                let v = unsafe { cmd.data.f32_ };
                self.tracking_vel_lookahead_s = if v.is_finite() { v.max(0.0) } else { self.tracking_vel_lookahead_s };
            }
            ParameterName::TRACKING_WHEEL_VEL_FB => {
                let v = unsafe { cmd.data.vec3_f32 };
                self.tracking_wheel_vel_fb = Vector3f::new(v[0], v[1], v[2]);
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
            ParameterName::POSE_FB_PIDII_LINEAR | ParameterName::POSE_FB_PIDII_ANGULAR => {
                let v = unsafe { cmd.data.vec5_f32 };
                let mut gain = self.pose_pid_controller.get_gain();
                if cmd.parameter_name == ParameterName::POSE_FB_PIDII_LINEAR {
                    for col in 0..5 {
                        gain[(0, col)] = v[col];
                        gain[(1, col)] = v[col];
                    }
                } else {
                    for col in 0..5 {
                        gain[(2, col)] = v[col];
                    }
                }
                self.pose_pid_controller.set_gain(gain);
            }
            _ => unreachable!(),
        }
    }
}
