use ateam_controls::{Vector3f, Vector4f};
use embassy_time::Duration;

//////////////////////////////////
//  trajectory tracking control  //
//////////////////////////////////

// Trajectory tracking feedback after TIGERs Mannheim `controlXY` / `controlW`
// (Firmware src/robot/ctrl_panthera.c, defaults from main2019/robot_impl.c,
// configCtrlDirectDrive). Position error is fed into the velocity setpoint (the
// wheel velocity loops do the correcting); the wheel torque path carries only the
// trajectory acceleration feed-forward. Inside the anti-jitter threshold both the
// feedback and the acceleration feed-forward are scaled linearly to zero.

/// [K (1/s), MAX (m/s), ANTI_JITTER (m)]: v_xy += clamp(K * pos_err_xy, MAX).
/// Radio-tunable as `POSE_FB_XY`.
// MAX lowered from TIGERs' 2.0 m/s while bringing up (a 0.3 m vision jump otherwise
// commands 2 m/s; see controls/analysis/feedback_sim.py).
pub const POSE_FB_XY: Vector3f = Vector3f::new(8.0, 0.3, 0.01);

/// [POS_K (1/s), POS_MAX (rad/s), ANTI_JITTER (rad), VEL_K]:
/// w += clamp(POS_K * theta_err, POS_MAX) + VEL_K * (w_traj - w_est).
/// Radio-tunable as `POSE_FB_W`.
// POS_MAX lowered from TIGERs' 8 rad/s while bringing up.
pub const POSE_FB_W: Vector4f = Vector4f::new(8.0, 2.0, 0.01, 2.0);

/// Wheel velocity loop on the motor controllers [KP (mA/(rad/s)), KI (mA/rad),
/// ANTI_JITTER (rad/s), MAX_CURRENT (A)]. P-only like TIGERs (velKi = 0).
/// MAX_CURRENT caps the total (feed-forward + loop) wheel current on the motor; the
/// motor's I2T budget keeps the long-term average at its continuous rating.
/// Radio-tunable as `WHEEL_VEL_FB`; periodically re-sent to the motors.
///
/// TIGERs' velKp = 300 mA per encoder tick per 1 ms (23040 ticks/rev, 0.27 rad/s per
/// tick), i.e. ~1.1 A/(rad/s), with the same DF45 motor. Our 4000 tick/rev encoder is
/// 1.57 rad/s per tick at 1 ms, so their gain would put ~1.7 A of quantization noise on
/// the current; KP = 200 keeps the same ~0.3 A per tick. Higher gains ring when a wheel
/// unloads or slips (measured: +/-1.2 A current std at 500 vs +/-0.2 A at 150-200).
pub const WHEEL_VEL_FB: Vector4f = Vector4f::new(200.0, 0.0, 1.5, 2.5);

/// Maximum feed-forward wheel current (A). The body acceleration output is
/// scaled uniformly so no wheel's feed-forward current exceeds this. Radio-tunable
/// as `WHEEL_FF_CURRENT_LIMIT`.
// Robot 2 cap sweep (plateau_profile.py): 2.0 A stalls sideways motion; 3.0 A gains
// nothing over 2.5 A (sideways is traction-limited, wheels spin out), and 3-4 A with
// kP 500 caused wheel slip and an unstable wheel loop. 2.5 A leaves transient headroom.
pub const WHEEL_FF_CURRENT_LIMIT_A: f32 = 2.5;

/// [ERROR_POS_LINEAR (m), ERROR_POS_ANGULAR (rad), ERROR_VEL_LINEAR (m/s), ERROR_VEL_ANGULAR (rad/s)]
/// Trajectory-vs-estimate errors above which closed-loop tracking is considered
/// diverged, triggering divergence recovery (active brake, then reset).
/// Radio-tunable as `TRACKING_DIVERGENCE_THRESHOLD`.
pub const TRACKING_DIVERGENCE_THRESHOLD: Vector4f = Vector4f::new(1.0, 1.57, 5.0, 10.0);

/// Maximum wheel current change per second for the feed-forward torque output
/// (TIGERs `CtrlPantheraUpdate`: 0.5 A per 1 ms control tick). Lowered to 50 A/s:
/// at TIGERs' rate the feed-forward steps at move start and Coulomb direction changes
/// spin up our least-loaded wheel (front-right/back-left diagonal) and yaw the robot.
pub const WHEEL_CURRENT_SLOPE_LIMIT_A_PER_S: f32 = 50.0;

/// Maximum magnitude of the body-level acceleration output [linear (m/s²), angular (rad/s²)].
/// The feed-forward output acceleration (local frame, friction and efficiency
/// applied) is clamped per axis to these limits before conversion to wheel torques.
/// Prevents runaway current commands from large PID errors.
pub const BODY_ACCEL_CLAMP_LINEAR: f32 = 20.0; // m/s²
pub const BODY_ACCEL_CLAMP_ANGULAR: f32 = 100.0; // rad/s²

/// Maximum magnitude of the body-level velocity output [linear (m/s), angular (rad/s)].
/// Clamped before conversion to wheel velocity setpoints.
pub const BODY_VEL_CLAMP_LINEAR: f32 = 10.0; // m/s
pub const BODY_VEL_CLAMP_ANGULAR: f32 = 5.0 * core::f32::consts::TAU; // rad/s (5 RPS), approx. 31.5 rad/s

/// IMU gyroscope full-scale ceiling (BMI088, 2000 deg/s range).
/// BODY_VEL_CLAMP_ANGULAR must stay below this — the KF cannot observe angular
/// velocities beyond this limit, so the state estimate would saturate and drift.
const IMU_GYRO_MAX_RAD_S: f32 = 2000.0 * core::f32::consts::PI / 180.0;
const _: () = assert!(
    BODY_VEL_CLAMP_ANGULAR <= IMU_GYRO_MAX_RAD_S,
    "BODY_VEL_CLAMP_ANGULAR exceeds IMU gyroscope maximum (2000 deg/s)"
);

/// Maximum linear speed (m/s) applied during SSL stop state.
/// Set below the 1.5 m/s rule limit to provide a safety buffer.
/// Applied after control policy output so feedback loops cannot overshoot.
pub const STOP_STATE_LINEAR_SPEED_LIMIT: f32 = 1.25; // m/s (SSL rule limit: 1.5 m/s)

/// Active brake controller parameters (halt and emergency-stop states).
/// Uses raw encoder velocity only — no KF state — so it is robust against
/// state estimator divergence.
///
/// Per-wheel braking current (A): I = clamp(-BRAKE_KP * ω, ±BRAKE_MAX_CURRENT_A)
///
/// Kp derivation: BRAKE_MAX_CURRENT_A / ω_expected_max, targeting current
/// saturation at the top of the expected play-speed range. With 1.0 A and
/// ~100 rad/s typical motor-shaft speed at play speed: Kp ≈ 0.01 A/(rad/s).
///
/// BRAKE_ANTI_JITTER_RADS: output is linearly scaled to zero below this
/// threshold (via PidController anti-jitter) to avoid fighting encoder
/// quantization noise near rest.
///
/// BRAKE_MAX_CURRENT_A must stay below MAX_CURRENT_MA / 1000 (5 A).
pub const BRAKE_KP: f32 = 0.01; // A / (rad/s)
pub const BRAKE_ANTI_JITTER_RADS: f32 = 5.0; // rad/s
pub const BRAKE_MAX_CURRENT_A: f32 = 1.0; // A

/// Trajectory divergence recovery: per-wheel speed (rad/s) below which all
/// wheels are considered stopped, and the number of consecutive ticks they must
/// stay below it for active braking to be "finished" (1 kHz, 1 tick = 1 ms). Once stopped, the controller resets and resumes from the fresh state
/// estimate.
pub const TRACKING_DIVERGENCE_RECOVERY_REST_WHEEL_VEL: f32 = 50.0; // rad/s
pub const TRACKING_DIVERGENCE_RECOVERY_REST_TICKS: u32 = 1;

/// Encoder lag compensation operating mode.
///
/// Intended progression: `Disabled` → `FeedforwardOnly` (validate model params)
///                        → `Full` (add KF correction once params are confirmed).
/// `KfCorrectionOnly` is available for diagnostics but replaces sensor data with model
/// predictions without improving command tracking — enable only with validated params.
#[derive(PartialEq, Eq)]
pub enum EncLagMode {
    /// No lag compensation. Default safe state.
    Disabled,
    /// Pre-compensates wheel commands via model inversion. Graceful degradation if
    /// model params are off — tracking suffers but the estimator is unaffected.
    FeedforwardOnly,
    /// Replaces encoder rows in the KF measurement with the model's predicted reading.
    /// Degrades state estimation if model params are wrong. Validate with
    /// `FeedforwardOnly` before enabling.
    KfCorrectionOnly,
    /// Both feedforward command compensation and KF measurement correction active.
    Full,
}

/// Active encoder lag compensation mode. Change this to enable/disable features.
pub const ENC_LAG_MODE: EncLagMode = EncLagMode::Disabled;

// Vision outlier gate parameters.
//
// BASE_RADIUS is derived from the worst-case prediction error between a valid
// vision measurement and the KF's dead-reckoned position:
//
//   base_radius = v_max * (N_miss * T_vision + T_latency) + slack
//
//   v_max      = 3.0 m/s  (DEFAULT_MAX_VEL_LINEAR)
//   N_miss     = 3         consecutive missed vision packets tolerated
//   T_vision   = 1/60 s   SSL-Vision camera rate
//   T_latency  = 0.150 s  measured worst-case camera→KF pipeline latency
//   slack      = 0.005 m  encoder dead-reckoning error over that window
//
// EXPAND_RATE controls how fast the gate grows when no valid update is received,
// allowing a robot that has been physically repositioned to eventually re-enter
// the gate. At 2.0 m/s the gate reaches the full half-field diagonal (~6 m)
// in ~3 s, which is realistic for a person carrying a robot across the field.
//
// SEED_SAMPLES and SEED_POS_STD_THRESH_M control the startup stability check:
// the robot must receive SEED_SAMPLES consecutive vision measurements whose
// positional standard deviation is below the threshold before the KF is seeded,
// implicitly requiring the robot to be near-stationary at boot.

// Encoder lag model physical parameters.
// K[i]:       DC gain per axis (dimensionless).
// T_SLOPE[i]: dT_eff/d|u| per axis [s / (m/s)].
// N_STEPS:    fixed-point inversion iterations.
pub const ENC_LAG_K: [f32; 2] = [0.95, 0.95];
pub const ENC_LAG_T_SLOPE: [f32; 2] = [0.095, 0.095];
pub const ENC_LAG_T_HORIZON: Duration = Duration::from_millis(10);
