use crate::motion::params::controller_params::{
    PoseAccelMode, PoseVelMode, ENC_LAG_K, ENC_LAG_MODE, ENC_LAG_T_HORIZON,
    ENC_LAG_T_SLOPE, POSE_ACCEL_MODE, POSE_VEL_MODE,
    TRACKING_DIVERGENCE_RECOVERY_REST_TICKS,
    TRACKING_DIVERGENCE_RECOVERY_REST_WHEEL_VEL
};
use crate::motion::pid::PidController;
use ateam_common_packets::bindings::{ParameterCommand, ParameterDataFormat, ParameterName};
use ateam_common_packets::radio::ManeuverCommand;
use ateam_controls::defaults::{DEFAULT_EKF_BUFF_LEN, DEFAULT_VISION_BUFF_LEN, EKF_INPUT_LEN, EKF_MEAS_LEN, EKF_STATE_LEN};
use ateam_controls::state_estimation::{BufferedEKF, StateEstimator, VisionFilter, VisionSample};
use ateam_controls::bangbang_trajectory::BangBangTraj3D;
use ateam_controls::linear_trajectory::LinearTrajectory;
use ateam_controls::pivot_trajectory::PivotTrajectory;
use ateam_controls::robot_model::{KalmanFilterParams, RobotModel, RobotPhysicalParams};
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
}

impl Trajectory for TrackedTrajectory {
    fn tick(&mut self, dt: f32) {
        match self {
            Self::BangBang(t) => t.tick(dt),
            Self::Pivot(t) => t.tick(dt),
            Self::Linear(t) => t.tick(dt),
        }
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        match self {
            Self::BangBang(t) => t.sample(),
            Self::Pivot(t) => t.sample(),
            Self::Linear(t) => t.sample(),
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
    /// Accel (torque) path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_accel_gain: Vector2f,
    /// Velocity setpoint path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_vel_gain: Vector2f,
    /// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
    pub tracking_error_thresh: Vector4f,
    /// [LINEAR_VEL_THRESHOLD, LINEAR_ACCEL_THRESHOLD, ANGULAR_VEL_THRESHOLD, ANGULAR_ACCEL_THRESHOLD]
    pub friction_comp_gating: Vector4f,
    /// Active trajectory. t=0 is always "now" (updated via tick each control tick).
    pub trajectory: Option<TrackedTrajectory>,
    /// Last command stored by `run_traj_track`, used to detect command changes.
    pub prev_cmd: Option<ManeuverCommand>,
    pub enc_lag: FirstOrderLag<2>,
    pub dt: f32,
    /// Cached KF state estimate — updated each tick before maneuver dispatch.
    pub state_estimate: Vector6f,
    pub wheels_disabled: bool,
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
            robot_model: RobotModel::new(
                dt,
                KalmanFilterParams::default(),
                RobotPhysicalParams::default(),
            )
            .expect("Failed to create RobotModel, check that parameters are valid"),
            state_estimator: StateEstimator::<DEFAULT_EKF_BUFF_LEN, DEFAULT_VISION_BUFF_LEN>::default(),
            pose_pid_controller: PidController::<3>::from_gains_matrix_with_anti_jitter(
                &controller_params::pose_pid_gains(),
                Some(controller_params::POSE_PID_ANTI_JITTER_THRESH),
            ),
            pose_accel_gain: controller_params::POSE_ACCEL_GAIN,
            pose_vel_gain: controller_params::POSE_VEL_GAIN,
            tracking_error_thresh: controller_params::TRACKING_ERROR_THRESHOLD,
            friction_comp_gating: controller_params::FRICTION_COMP_GATING,
            trajectory: None,
            prev_cmd: None,
            enc_lag,
            dt,
            state_estimate: Vector6f::zeros(),
            wheels_disabled: true,
            tracking_divergence_state: TrackingDivergenceState::Normal,
            tracking_recovery_at_rest_ticks: 0,
        }
    }

    pub fn vision_active(&self) -> bool {
        self.state_estimator.vision_active()
    }

    pub fn reset(&mut self) {
        self.state_estimator.init(
            SVector::<f32, 3>::zeros(),
            SVector::<f32, 3>::zeros(),
        );
        self.robot_model.reset();
        self.pose_pid_controller.reset();
        self.trajectory = None;
        self.prev_cmd = None;
        self.enc_lag.reset();
        self.wheels_disabled = true;
        self.tracking_divergence_state = TrackingDivergenceState::Normal;
        self.tracking_recovery_at_rest_ticks = 0;
    }

    /// Clear trajectory and command history without touching the PID or KF.
    ///
    /// Called by `ManeuverManager` on mode change to ensure the next tick
    /// replans from the current state estimate.
    pub fn reset_trajectory(&mut self) {
        self.trajectory = None;
        self.prev_cmd = None;
    }

    /// Run KF vision handling, measurement construction, and state update.
    /// Returns the pre-update state prediction used for telemetry.
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

        self.state_estimator.tick(
            Instant::now().as_micros(),
            &imu,
            &wheel_vel_meas,
            vision,
        ).map_err(|_| ControlsError::SingularMatrix)?;

        let mut state_prediction = SVector::<f32, 6>::zeros();
        state_prediction.fixed_rows_mut::<3>(0).copy_from(&self.state_estimator.get_pos_buff());
        state_prediction.fixed_rows_mut::<3>(3).copy_from(&self.state_estimator.get_vel_buff());

        let mut state_est = SVector::<f32, 6>::zeros();
        state_est.fixed_rows_mut::<3>(0).copy_from(&self.state_estimator.get_pos());
        state_est.fixed_rows_mut::<3>(3).copy_from(&self.state_estimator.get_vel());
        self.state_estimate = state_est;

        Ok(state_prediction)
    }

    /// Compute global-frame friction force from current state and commanded body twist/accel.
    pub fn compute_friction(&self, body_twist: Vector3f, body_accel: Vector3f) -> Vector3f {
        let state_estimate = self.state_estimate;
        let theta = state_estimate.z;
        let r_glob_to_loc = z_rotation_mat(-theta);
        let r_loc_to_glob = z_rotation_mat(theta);

        let est_twist_global: Vector3f = state_estimate.fixed_rows::<3>(3).into();
        let est_twist_local: Vector3f = r_glob_to_loc * est_twist_global;
        let tgt_twist_local: Vector3f = r_glob_to_loc * body_twist;
        let cmd_accel_local: Vector3f = r_glob_to_loc * body_accel;

        let lin_vel_mag = hypotf(est_twist_local.x, est_twist_local.y);
        let lin_accel_mag = hypotf(cmd_accel_local.x, cmd_accel_local.y);
        let ang_vel_mag = fabsf(est_twist_local.z);
        let ang_accel_mag = fabsf(cmd_accel_local.z);

        let linear_comp_on = lin_accel_mag >= self.friction_comp_gating[1]
            || lin_vel_mag >= self.friction_comp_gating[0];
        let angular_comp_on = ang_accel_mag >= self.friction_comp_gating[3]
            || ang_vel_mag >= self.friction_comp_gating[2];

        let fric_twist_local = Vector3f::new(
            if linear_comp_on {
                tgt_twist_local.x
            } else {
                0.0
            },
            if linear_comp_on {
                tgt_twist_local.y
            } else {
                0.0
            },
            if angular_comp_on {
                tgt_twist_local.z
            } else {
                0.0
            },
        );

        let friction_force_local = self.robot_model.compute_friction_force(fric_twist_local);
        r_loc_to_glob * friction_force_local
    }

    // -----------------------------------------------------------------------
    // Trajectory management and tracking
    // -----------------------------------------------------------------------

    /// Trajectory-divergence recovery state machine, run once per control tick
    /// after the maneuver dispatch. Returns `true` while the controller is
    /// recovering, signalling the control task to command the active brake.
    ///
    /// - Normal: a large unexpected tracking error (an active trajectory whose
    ///   error exceeds `TRAJ_RECOMPUTE_ERROR`, e.g. a collision knocking the
    ///   robot off course) trips into `Recovering`.
    /// - Recovering: active braking is commanded until all wheels stay below
    ///   `RECOVERY_AT_REST_WHEEL_SPEED` for `RECOVERY_AT_REST_TICKS`, then the
    ///   controller is reset (replanning from the fresh state estimate) and
    ///   normal operation resumes. Uses raw encoder wheel speeds, not the KF
    ///   estimate, since divergence implies the estimate is unreliable.
    pub fn update_tracking_divergence_recovery(&mut self, wheel_vel_meas: Vector4f) -> bool {
        match self.tracking_divergence_state {
            TrackingDivergenceState::Normal => {
                if self.trajectory.is_some() && self.tracking_error_exceeded() {
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
    /// - An existing trajectory  → seed from its current `sample()` state
    ///   (continuous handoff). Snapping to `state_estimate` would mask
    ///   divergence from the recovery state machine.
    /// - No trajectory          → seed from `state_estimate` (snap to reality).
    pub fn run_traj_track<F>(
        &mut self,
        cmd: ManeuverCommand,
        make_traj: F,
    ) -> Result<ManeuverSetpoints, ControlsError>
    where
        F: FnOnce(Vector6f) -> Result<TrackedTrajectory, ControlsError>,
    {
        let should_replan = self.trajectory.is_none() || self.command_changed(&cmd);

        if should_replan {
            let seed = match self.trajectory.as_ref() {
                // Existing trajectory → continuous seed from its current state
                // (before this tick's advance), even if the tracking error is
                // high. Never snap to the estimate here: doing so would zero the
                // tracking error and prevent the divergence-recovery state
                // machine from ever detecting a collision.
                Some(traj) => traj.sample().0,
                // No trajectory yet → seed from the state estimate.
                None => self.state_estimate,
            };
            match make_traj(seed) {
                Ok(new_traj) => {
                    self.trajectory = Some(new_traj);
                }
                Err(e) => {
                    self.trajectory = None;
                    return Err(e);
                }
            }
        }

        self.prev_cmd = Some(cmd);

        let traj = self
            .trajectory
            .as_ref()
            .ok_or(ControlsError::InvalidInput)?;
        let (traj_state, traj_accel) = traj.sample();

        let setpoints = self.calc_tracking_setpoints(traj_state, traj_accel)?;

        // Advance the trajectory's internal clock by dt.
        // After this, traj.sample() returns the state at the start of the next tick.
        if let Some(traj) = &mut self.trajectory {
            traj.tick(self.dt);
        }

        Ok(setpoints)
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// PID+FF trajectory tracking: compute body-frame setpoints from the
    /// trajectory's current position/velocity/acceleration sample.
    fn calc_tracking_setpoints(
        &mut self,
        traj_state: Vector6f,
        traj_accel: Vector3f,
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

        let accel_out: Vector3f = {
            let accel_ff_term = if matches!(
                POSE_ACCEL_MODE,
                PoseAccelMode::FeedforwardOnly | PoseAccelMode::Full
            ) {
                self.pose_accel_gain[0] * traj_accel
            } else {
                Vector3f::zeros()
            };

            let accel_fb_term = if matches!(
                POSE_ACCEL_MODE,
                PoseAccelMode::FeedbackOnly | PoseAccelMode::Full
            ) {
                self.pose_accel_gain[1] * pos_pid_feedback
            } else {
                Vector3f::zeros()
            };

            accel_ff_term + accel_fb_term
        };

        let twist_out: Vector3f = {
            let vel_ff_term: Vector3f = if matches!(
                POSE_VEL_MODE,
                PoseVelMode::FeedforwardOnly | PoseVelMode::Full
            ) {
                self.pose_vel_gain[0] * traj_vel
            } else {
                (state_estimate.fixed_rows::<3>(3) + accel_out * self.dt).into()
            };

            let vel_fb_term =
                if matches!(POSE_VEL_MODE, PoseVelMode::FeedbackOnly | PoseVelMode::Full) {
                    self.pose_vel_gain[1] * pos_pid_feedback * self.dt
                } else {
                    Vector3f::zeros()
                };

            vel_ff_term + vel_fb_term
        };

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
        linear_pose_error > self.tracking_error_thresh[0]
            || angular_pose_error > self.tracking_error_thresh[1]
            || linear_twist_error > self.tracking_error_thresh[2]
            || angular_twist_error > self.tracking_error_thresh[3]
    }

    // -----------------------------------------------------------------------
    // Parameter read/write (data operations only — reset is caller's concern)
    // -----------------------------------------------------------------------

    pub fn expected_format(name: ParameterName::Type) -> Option<ParameterDataFormat::Type> {
        match name {
            ParameterName::KF_PROCESS_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MEASUREMENT_STD => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::KF_MAX_STATE => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_WHEEL => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::PHYS_INERTIA => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_MOTOR_MODEL => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::PHYS_FRICTION_MODEL => Some(ParameterDataFormat::VEC6_F32),
            ParameterName::FRICTION_COMP_GATING => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_CONTROL_GAIN => Some(ParameterDataFormat::VEC2_F32),
            ParameterName::TRAJ_RECOMPUTE_ERROR => Some(ParameterDataFormat::VEC4_F32),
            ParameterName::POSE_FB_PIDII_LINEAR => Some(ParameterDataFormat::VEC5_F32),
            ParameterName::POSE_FB_PIDII_ANGULAR => Some(ParameterDataFormat::VEC5_F32),
            _ => None,
        }
    }

    pub fn read_param(&self, name: ParameterName::Type, reply: &mut ParameterCommand) {
        let kf = &self.robot_model.kf_params;
        let phys = &self.robot_model.physical_params;
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
            ParameterName::KF_MAX_STATE => {
                reply.data.vec4_f32 = [
                    kf.max_pos_linear,
                    kf.max_pos_angular,
                    kf.max_vel_linear,
                    kf.max_vel_angular,
                ];
            }
            ParameterName::PHYS_WHEEL => {
                reply.data.vec4_f32 = [phys.alpha, phys.beta, phys.l, phys.r];
            }
            ParameterName::PHYS_INERTIA => {
                reply.data.vec2_f32 = [phys.mass, phys.iz];
            }
            ParameterName::PHYS_MOTOR_MODEL => {
                reply.data.vec2_f32 = [phys.motor_torque_constant, phys.motor_efficiency_factor];
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                reply.data.vec6_f32 = [
                    phys.coulomb_friction_coefficient_linear_x,
                    phys.coulomb_friction_coefficient_linear_y,
                    phys.coulomb_friction_coefficient_angular,
                    phys.viscous_friction_coefficient_linear_x,
                    phys.viscous_friction_coefficient_linear_y,
                    phys.viscous_friction_coefficient_angular,
                ];
            }
            ParameterName::FRICTION_COMP_GATING => {
                reply.data.vec4_f32 = self.friction_comp_gating.into();
            }
            ParameterName::POSE_CONTROL_GAIN => {
                reply.data.vec2_f32 = self.pose_accel_gain.into();
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                reply.data.vec4_f32 = self.tracking_error_thresh.into();
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
                self.state_estimator = StateEstimator::new(
                    ekf,
                    vision_filter,
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
                self.state_estimator = StateEstimator::new(
                    ekf,
                    vision_filter,
                );
                self.reset();
            }
            ParameterName::KF_MAX_STATE => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut kf = self.robot_model.kf_params;
                kf.max_pos_linear = v[0];
                kf.max_pos_angular = v[1];
                kf.max_vel_linear = v[2];
                kf.max_vel_angular = v[3];
                self.robot_model.update_kf_params(kf);
            }
            ParameterName::PHYS_WHEEL => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut p = self.robot_model.physical_params;
                p.alpha = v[0];
                p.beta = v[1];
                p.l = v[2];
                p.r = v[3];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::PHYS_INERTIA => {
                let v = unsafe { cmd.data.vec2_f32 };
                let mut p = self.robot_model.physical_params;
                p.mass = v[0];
                p.iz = v[1];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::PHYS_MOTOR_MODEL => {
                let v = unsafe { cmd.data.vec2_f32 };
                let mut p = self.robot_model.physical_params;
                p.motor_torque_constant = v[0];
                p.motor_efficiency_factor = v[1];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::PHYS_FRICTION_MODEL => {
                let v = unsafe { cmd.data.vec6_f32 };
                let mut p = self.robot_model.physical_params;
                p.coulomb_friction_coefficient_linear_x = v[0];
                p.coulomb_friction_coefficient_linear_y = v[1];
                p.coulomb_friction_coefficient_angular = v[2];
                p.viscous_friction_coefficient_linear_x = v[3];
                p.viscous_friction_coefficient_linear_y = v[4];
                p.viscous_friction_coefficient_angular = v[5];
                let _ = self.robot_model.update_physical_params(p);
            }
            ParameterName::FRICTION_COMP_GATING => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.friction_comp_gating = Vector4f::new(v[0], v[1], v[2], v[3]);
            }
            ParameterName::POSE_CONTROL_GAIN => {
                let v = unsafe { cmd.data.vec2_f32 };
                self.pose_accel_gain = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.tracking_error_thresh = Vector4f::new(v[0], v[1], v[2], v[3]);
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
