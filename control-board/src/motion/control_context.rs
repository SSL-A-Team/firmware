use crate::motion::params::controller_params::{
    EncLagMode, PoseAccelMode, PoseVelMode, ENC_LAG_K, ENC_LAG_MODE, ENC_LAG_T_HORIZON,
    ENC_LAG_T_SLOPE, POSE_ACCEL_MODE, POSE_VEL_MODE, VISION_GATE_BASE_RADIUS_M,
    VISION_GATE_EXPAND_RATE_M_PER_S, VISION_SEED_POS_STD_THRESH_M, VISION_SEED_SAMPLES,
};
use crate::motion::pid::PidController;
use ateam_common_packets::bindings::{ParameterCommand, ParameterDataFormat, ParameterName};
use ateam_common_packets::radio::ManeuverCommand;
use ateam_controls::bangbang_trajectory::BangBangTraj3D;
use ateam_controls::pivot_trajectory::PivotTrajectory;
use ateam_controls::robot_model::{KalmanFilterParams, RobotModel, RobotPhysicalParams};
use ateam_controls::trajectory::Trajectory;
use ateam_controls::{
    z_rotation_mat, ControlsError, Vector2f, Vector3f, Vector4f, Vector6f, Vector8f,
};
use ateam_lib_stm32::model::{FirstOrderLag, FirstOrderLagParams};
use core::f32::consts::PI;
use embassy_time::Duration;
use libm::{fabsf, hypotf, remainderf, sqrtf};
use nalgebra::SVector;

pub(crate) const VISION_ACTIVE_TIMEOUT_S: f32 = 0.5;

/// All trajectory types the firmware can track.
///
/// Provides static dispatch over trajectory types that each implement the
/// `Trajectory` trait directly (owning their own state).
pub enum TrackedTrajectory {
    BangBang(BangBangTraj3D),
    Pivot(PivotTrajectory),
}

impl Trajectory for TrackedTrajectory {
    fn tick(&mut self, dt: f32) {
        match self {
            Self::BangBang(t) => t.tick(dt),
            Self::Pivot(t) => t.tick(dt),
        }
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        match self {
            Self::BangBang(t) => t.sample(),
            Self::Pivot(t) => t.sample(),
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

/// Startup seeding and steady-state tracking state for the vision outlier gate.
///
/// On boot the gate starts in `Seeding`, collecting vision samples until the
/// position estimates are stable enough to trust. Once seeded it transitions to
/// `Tracking`, where each incoming vision measurement is compared against the
/// KF's predicted position. Measurements outside the gate are rejected as
/// outliers; the gate expands over time so a robot that has been physically
/// repositioned can eventually re-enter and be accepted.
pub enum VisionGateState {
    /// Collecting initial vision samples. Transitions to `Tracking` once
    /// `VISION_SEED_SAMPLES` samples have been received and their positional
    /// standard deviation is below `VISION_SEED_POS_STD_THRESH_M`.
    Seeding {
        n: u32,
        pos_sum: Vector2f,
        pos_sq_sum: Vector2f,
    },
    /// Gate is active. Measurements within `VISION_GATE_BASE_RADIUS_M +
    /// VISION_GATE_EXPAND_RATE_M_PER_S * time_since_last_valid_s` of the KF
    /// predicted position are accepted; others are rejected and the expansion
    /// timer advances.
    Tracking { time_since_last_valid_s: f32 },
}

impl Default for VisionGateState {
    fn default() -> Self {
        VisionGateState::Seeding {
            n: 0,
            pos_sum: Vector2f::zeros(),
            pos_sq_sum: Vector2f::zeros(),
        }
    }
}

/// Notable vision gate events reported to the caller each tick via
/// `ControlContext::last_gate_event`. Reset to `None` on normal ticks.
/// Used by `control_task` to emit `ErrorTelemetry` over radio.
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum VisionGateEvent {
    None,
    /// Seed stability check failed — robot was not stationary, restarting.
    SeedReset,
    /// First outlier rejection after a valid tracking period (start of a burst).
    /// Subsequent rejections in the same burst are suppressed to avoid spam.
    FirstReject,
    /// Measurement accepted after gate expansion — robot was physically repositioned.
    AcceptJump,
}

/// Internal gate decision for a single vision tick. Computed with only
/// `vision_gate` borrowed, then applied once that borrow ends.
enum GateAction {
    /// Still accumulating seed samples — no update to KF.
    Accumulate,
    /// Seed samples collected but variance check failed — restart accumulator.
    SeedReset,
    /// Seed stable — snap KF to vision and transition to Tracking.
    SeedComplete,
    /// Measurement within base radius — pass to KF update normally.
    Accept,
    /// Measurement outside base radius but within expanded gate — snap KF
    /// to vision and reset velocity to encoder-implied.
    AcceptJump,
    /// First outlier rejection since the last valid update — emit telemetry once.
    FirstReject,
    /// Subsequent outlier rejection in the same burst — suppress telemetry.
    Reject,
    /// No vision packet this tick.
    NoVision,
}

/// Controller infrastructure passed into each maneuver on every tick.
///
/// Owns the KF/robot-model, trajectory state, PID, and encoder-lag model.
/// Maneuvers borrow this via `&mut ControlContext` to call planning and
/// tracking helpers.
pub struct ControlContext {
    pub robot_model: RobotModel,
    pub pose_pid_controller: PidController<3>,
    /// Jerk (acceleration slew-rate) clamp for the FB pose controller output:
    /// [linear (m/s³), angular (rad/s³)]. Tunable live via the
    /// `POSE_CONTROL_GAIN` parameter (repurposed). The FF/FB pose accel gains
    /// are hardcoded to 1.0.
    pub jerk_clamp: Vector2f,
    /// Velocity setpoint path gains: [FEEDFORWARD_GAIN, FEEDBACK_GAIN]
    pub pose_vel_gain: Vector2f,
    /// [ERROR_POS_LINEAR, ERROR_POS_ANGULAR, ERROR_VEL_LINEAR, ERROR_VEL_ANGULAR]
    pub traj_recompute_error: Vector4f,
    /// [LINEAR_VEL_THRESHOLD, LINEAR_ACCEL_THRESHOLD, ANGULAR_VEL_THRESHOLD, ANGULAR_ACCEL_THRESHOLD]
    pub friction_comp_gating: Vector4f,
    /// Active trajectory. t=0 is always "now" (updated via tick each control tick).
    pub trajectory: Option<TrackedTrajectory>,
    /// Last command stored by `run_traj_track`, used to detect command changes.
    pub prev_cmd: Option<ManeuverCommand>,
    pub enc_lag: FirstOrderLag<2>,
    pub dt: f32,
    /// Previous (jerk-limited) acceleration output of the FB pose controller.
    /// Used to slew-rate limit the acceleration output between control ticks.
    pub prev_accel_out: Vector3f,
    /// Cached KF state estimate — updated each tick before maneuver dispatch.
    pub state_estimate: Vector6f,
    pub time_since_vision_update_s: f32,
    pub vision_gate: VisionGateState,
    /// Notable gate event from the most recent `update_state_estimate` call.
    /// Reset to `None` every tick; set when a reportable condition occurs.
    pub last_gate_event: VisionGateEvent,
    pub wheels_disabled: bool,
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
            pose_pid_controller: PidController::<3>::from_gains_matrix_with_anti_jitter(
                &controller_params::pose_pid_gains(),
                Some(controller_params::POSE_PID_ANTI_JITTER_THRESH),
            ),
            jerk_clamp: Vector2f::new(
                controller_params::JERK_CLAMP_LINEAR,
                controller_params::JERK_CLAMP_ANGULAR,
            ),
            pose_vel_gain: controller_params::POSE_VEL_GAIN,
            traj_recompute_error: controller_params::TRAJ_RECOMPUTE_ERROR,
            friction_comp_gating: controller_params::FRICTION_COMP_GATING,
            trajectory: None,
            prev_cmd: None,
            enc_lag,
            dt,
            prev_accel_out: Vector3f::zeros(),
            state_estimate: Vector6f::zeros(),
            time_since_vision_update_s: VISION_ACTIVE_TIMEOUT_S + 1.0,
            vision_gate: VisionGateState::default(),
            last_gate_event: VisionGateEvent::None,
            wheels_disabled: true,
        }
    }

    pub fn vision_active(&self) -> bool {
        self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S
    }

    pub fn reset(&mut self) {
        self.robot_model.reset();
        self.pose_pid_controller.reset();
        self.trajectory = None;
        self.prev_cmd = None;
        self.time_since_vision_update_s = VISION_ACTIVE_TIMEOUT_S + 1.0;
        self.vision_gate = VisionGateState::default();
        self.last_gate_event = VisionGateEvent::None;
        self.enc_lag.reset();
        self.prev_accel_out = Vector3f::zeros();
        self.wheels_disabled = true;
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
        vision_update: bool,
        wheel_vel_meas: Vector4f,
        imu_gyro_theta_meas: f32,
    ) -> Result<Vector6f, ControlsError> {
        // Capture the KF's current predicted position before any snap so the gate
        // compares against dead-reckoned state, not a freshly-overwritten value.
        let predicted_state = self.robot_model.get_state();

        // [1] Outlier gate: classify this vision tick.
        //
        // Gate is XY-only. Theta outliers are not rejected here because heading
        // errors from wheel slip are common and the controller handles them.
        //
        // Accumulator mutations happen inside the match; KF and state-machine
        // mutations happen after so `vision_gate` is not borrowed when calling
        // into `robot_model`.
        let gate_action = if vision_update {
            match &mut self.vision_gate {
                VisionGateState::Seeding {
                    n,
                    pos_sum,
                    pos_sq_sum,
                } => {
                    *n += 1;
                    pos_sum.x += vision_pose_meas.x;
                    pos_sum.y += vision_pose_meas.y;
                    pos_sq_sum.x += vision_pose_meas.x * vision_pose_meas.x;
                    pos_sq_sum.y += vision_pose_meas.y * vision_pose_meas.y;

                    if *n >= VISION_SEED_SAMPLES {
                        let nf = *n as f32;
                        let mean_x = pos_sum.x / nf;
                        let mean_y = pos_sum.y / nf;
                        // Variance = E[x²] - (E[x])²
                        let var_x = pos_sq_sum.x / nf - mean_x * mean_x;
                        let var_y = pos_sq_sum.y / nf - mean_y * mean_y;
                        if sqrtf(var_x) < VISION_SEED_POS_STD_THRESH_M
                            && sqrtf(var_y) < VISION_SEED_POS_STD_THRESH_M
                        {
                            GateAction::SeedComplete
                        } else {
                            GateAction::SeedReset
                        }
                    } else {
                        GateAction::Accumulate
                    }
                }
                VisionGateState::Tracking {
                    time_since_last_valid_s,
                } => {
                    let dist = hypotf(
                        vision_pose_meas.x - predicted_state[0],
                        vision_pose_meas.y - predicted_state[1],
                    );
                    // Gate radius expands linearly while no valid update is received,
                    // allowing a physically repositioned robot to eventually re-enter.
                    let gate_radius = VISION_GATE_BASE_RADIUS_M
                        + VISION_GATE_EXPAND_RATE_M_PER_S * *time_since_last_valid_s;

                    if dist < gate_radius {
                        *time_since_last_valid_s = 0.0;
                        if dist >= VISION_GATE_BASE_RADIUS_M {
                            GateAction::AcceptJump
                        } else {
                            GateAction::Accept
                        }
                    } else {
                        // Distinguish first rejection from subsequent ones so
                        // the caller can emit a single telemetry burst event.
                        let is_first = *time_since_last_valid_s == 0.0;
                        *time_since_last_valid_s += self.dt;
                        if is_first {
                            GateAction::FirstReject
                        } else {
                            GateAction::Reject
                        }
                    }
                }
            }
        } else {
            // No vision packet — advance the expansion timer so a robot that
            // loses vision entirely can still recover when it returns.
            if let VisionGateState::Tracking {
                time_since_last_valid_s,
            } = &mut self.vision_gate
            {
                *time_since_last_valid_s += self.dt;
            }
            GateAction::NoVision
        };

        // [2] Apply gate decision: KF snaps and state-machine transitions.
        //
        // SeedComplete and AcceptJump both snap position and velocity. SeedComplete
        // additionally transitions the gate to Tracking. AcceptJump drops any
        // in-flight trajectory so the controller replans from the new pose
        // instead of continuing to chase the old one.
        self.last_gate_event = VisionGateEvent::None;
        let effective_vision_update = match gate_action {
            GateAction::SeedComplete | GateAction::AcceptJump => {
                self.robot_model.kf_set_pose(vision_pose_meas);
                // Snap KF velocity to directly-measured values.
                // Linear (vx, vy): encoder-implied via wheel Jacobian.
                // Angular (ω): gyro, bypassing the Jacobian — lower noise
                // (0.015 rad/s vs 50 rad/s encoder std) and no wheel-slip error.
                let mut vel_seed =
                    self.robot_model.transform_wheel2twist(vision_pose_meas.z) * wheel_vel_meas;
                vel_seed[2] = imu_gyro_theta_meas;
                self.robot_model.kf_set_vel(vel_seed);
                self.trajectory = None;
                self.prev_cmd = None;
                self.pose_pid_controller.reset();
                if matches!(gate_action, GateAction::SeedComplete) {
                    self.vision_gate = VisionGateState::Tracking {
                        time_since_last_valid_s: 0.0,
                    };
                    defmt::info!("vision gate: seeding complete");
                } else {
                    self.last_gate_event = VisionGateEvent::AcceptJump;
                    defmt::info!("vision gate: large jump accepted");
                }
                true
            }
            GateAction::SeedReset => {
                self.vision_gate = VisionGateState::default();
                self.last_gate_event = VisionGateEvent::SeedReset;
                defmt::warn!("vision gate: seed stability check failed, restarting");
                false
            }
            GateAction::FirstReject => {
                self.last_gate_event = VisionGateEvent::FirstReject;
                defmt::warn!("vision gate: outlier rejected");
                false
            }
            GateAction::Accept => true,
            GateAction::Reject => {
                defmt::warn!("vision gate: outlier rejected");
                false
            }
            GateAction::Accumulate | GateAction::NoVision => false,
        };

        // [3] Update the vision activity timer used by global-position maneuvers.
        if effective_vision_update {
            self.time_since_vision_update_s = 0.0;
        } else if self.time_since_vision_update_s <= VISION_ACTIVE_TIMEOUT_S {
            self.time_since_vision_update_s += self.dt;
        }

        // Capture post-snap / pre-KF-update state for telemetry.
        let state_prediction = self.robot_model.get_state();

        let measurement: Vector8f = if matches!(
            ENC_LAG_MODE,
            EncLagMode::KfCorrectionOnly | EncLagMode::Full
        ) {
            let lag_state = self.enc_lag.state();
            let lag_body = Vector3f::new(lag_state.x, lag_state.y, imu_gyro_theta_meas);
            let lag_wheel = self.robot_model.transform_twist2wheel(state_prediction[2]) * lag_body;
            nalgebra::vector![
                vision_pose_meas.x,
                vision_pose_meas.y,
                vision_pose_meas.z,
                lag_wheel.x,
                lag_wheel.y,
                lag_wheel.z,
                lag_wheel.w,
                imu_gyro_theta_meas,
            ]
        } else {
            nalgebra::vector![
                vision_pose_meas.x,
                vision_pose_meas.y,
                vision_pose_meas.z,
                wheel_vel_meas.x,
                wheel_vel_meas.y,
                wheel_vel_meas.z,
                wheel_vel_meas.w,
                imu_gyro_theta_meas,
            ]
        };

        self.robot_model
            .kf_update(measurement, !effective_vision_update, false, false)?;
        self.state_estimate = self.robot_model.get_state();

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

    /// Returns `true` when a trajectory exists and the tracking error is within
    /// the configured thresholds.  Used by `run_traj_track` to decide the
    /// replan seed.
    pub fn traj_tracking_healthy(&self) -> bool {
        self.trajectory.is_some() && !self.tracking_error_exceeded()
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
    /// - the tracking error exceeds the configured thresholds, OR
    /// - `cmd` differs from the last stored command.
    ///
    /// # Seed selection
    /// - Tracking healthy but command changed  → seed from the trajectory's
    ///   current `sample()` state (continuous handoff).
    /// - No trajectory or error exceeded       → seed from `state_estimate`
    ///   (snap to reality).
    pub fn run_traj_track<F>(
        &mut self,
        cmd: ManeuverCommand,
        make_traj: F,
    ) -> Result<ManeuverSetpoints, ControlsError>
    where
        F: FnOnce(Vector6f) -> Result<TrackedTrajectory, ControlsError>,
    {
        let should_replan = !self.traj_tracking_healthy() || self.command_changed(&cmd);

        if should_replan {
            let seed = if self.traj_tracking_healthy() {
                // Command changed but tracking healthy → continuous seed from
                // the trajectory's current state (before this tick's advance).
                self.trajectory.as_ref().unwrap().sample().0
            } else {
                // No trajectory or error exceeded → snap to estimate.
                self.state_estimate
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
                traj_accel
            } else {
                Vector3f::zeros()
            };

            let accel_fb_term = if matches!(
                POSE_ACCEL_MODE,
                PoseAccelMode::FeedbackOnly | PoseAccelMode::Full
            ) {
                pos_pid_feedback
            } else {
                Vector3f::zeros()
            };

            accel_ff_term + accel_fb_term
        };

        // Jerk-limit (slew-rate limit) the FB pose controller's acceleration
        // output. The acceleration cannot change by more than `JERK_CLAMP * dt`
        // per control tick, smoothing the commanded body acceleration (and the
        // resulting wheel torques) at the cost of a small amount of
        // responsiveness on large step changes.
        let accel_out = {
            let max_delta_lin = self.jerk_clamp[0] * self.dt;
            let max_delta_ang = self.jerk_clamp[1] * self.dt;
            let delta = accel_out - self.prev_accel_out;
            let limited = Vector3f::new(
                self.prev_accel_out.x + delta.x.clamp(-max_delta_lin, max_delta_lin),
                self.prev_accel_out.y + delta.y.clamp(-max_delta_lin, max_delta_lin),
                self.prev_accel_out.z + delta.z.clamp(-max_delta_ang, max_delta_ang),
            );
            self.prev_accel_out = limited;
            limited
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
        linear_pose_error > self.traj_recompute_error[0]
            || angular_pose_error > self.traj_recompute_error[1]
            || linear_twist_error > self.traj_recompute_error[2]
            || angular_twist_error > self.traj_recompute_error[3]
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
                reply.data.vec4_f32 = [
                    kf.process_noise_std_pos_linear,
                    kf.process_noise_std_pos_angular,
                    kf.process_noise_std_vel_linear,
                    kf.process_noise_std_vel_angular,
                ];
            }
            ParameterName::KF_MEASUREMENT_STD => {
                reply.data.vec4_f32 = [
                    kf.measurement_noise_std_vision_pos_linear,
                    kf.measurement_noise_std_vision_pos_angular,
                    kf.measurement_noise_std_encoder_vel_angular,
                    kf.measurement_noise_std_gyro_vel_angular,
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
                reply.data.vec2_f32 = self.jerk_clamp.into();
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                reply.data.vec4_f32 = self.traj_recompute_error.into();
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
                let v = unsafe { cmd.data.vec4_f32 };
                let mut kf = self.robot_model.kf_params;
                kf.process_noise_std_pos_linear = v[0];
                kf.process_noise_std_pos_angular = v[1];
                kf.process_noise_std_vel_linear = v[2];
                kf.process_noise_std_vel_angular = v[3];
                self.robot_model.update_kf_params(kf);
            }
            ParameterName::KF_MEASUREMENT_STD => {
                let v = unsafe { cmd.data.vec4_f32 };
                let mut kf = self.robot_model.kf_params;
                kf.measurement_noise_std_vision_pos_linear = v[0];
                kf.measurement_noise_std_vision_pos_angular = v[1];
                kf.measurement_noise_std_encoder_vel_angular = v[2];
                kf.measurement_noise_std_gyro_vel_angular = v[3];
                self.robot_model.update_kf_params(kf);
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
                self.jerk_clamp = Vector2f::new(v[0], v[1]);
            }
            ParameterName::TRAJ_RECOMPUTE_ERROR => {
                let v = unsafe { cmd.data.vec4_f32 };
                self.traj_recompute_error = Vector4f::new(v[0], v[1], v[2], v[3]);
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
