#pragma once

#include <stdbool.h>
#include <stdint.h>

// motor_config.h is provided per-binary and defines:
//   MOTOR_POLE_PAIRS         - rotor pole pair count (compile-time constant for velocity calc)
//   INVERT_MOTOR_DIRECTION   - flip commutation sense (bool)
//   MOTOR_KE_MV_PER_DRAD_Q8  - back-EMF constant, mV per deci-rad/s, UQ8.8
//   MOTOR_R_LOOP_MOHM        - drive loop resistance (windings + 2 FETs + shunt), milliohms
#include "motor_config.h"
#include "pid.h"
// per-binary config. Provides F_SYS_CLK_HZ, which NUM_RAW_DC_STEPS below needs,
// and the CS_SYNC_SAMPLING gate.
#include "system.h"

#define MAX_DUTYCYCLE_COMMAND 4095U
#define MIN_DUTYCYCLE_COMMAND -(MAX_DUTYCYCLE_COMMAND)

// period 20833ns
#define PWM_FREQ_HZ 40000    // if you update date, be conscious of dead time ratio
#define PWM_TIM_PRESCALER 0  // you almost certainly don't want to touch this with the low-ish sys clk of 48MHz

#define BATTERY_VOLTAGE_MV (25200U)

//////////////////////
//  ERROR HANDLING  //
//////////////////////

#define HALL_DISCONNECT_ERROR_INCREMENT 10
#define HALL_DISCONNECT_ERROR_CLEAR_DECREMENT 1
#define HALL_DISCONNECT_MAX_ACCU_ERROR 1000
#define HALL_DISCONNECT_ERROR_THRESHOLD 50

#define HALL_POWER_ERROR_INCREMENT 10
#define HALL_POWER_ERROR_CLEAR_DECREMENT 1
#define HALL_POWER_MAX_ACCU_ERROR 1000
#define HALL_POWER_ERROR_THRESHOLD 50

#define HALL_TRANSITION_ERROR_THRESHOLD 3

typedef struct MotorErrors {
    bool hall_power;
    bool hall_disconnected;
    bool invalid_transitions;
    bool commutation_watchdog_timeout;
} MotorErrors_t;

////////////////////////////////
//  LOW LEVEL CONTROL PARAMS  //
////////////////////////////////

// no div on CK_INT = 48MHz
// Tdts = CK_INT = 48MHz
// xxx yyyyy -> 0xx selects no multiplier
// Tdts = 20.8 ns
// yyyyy = 00111 = 7
// DEAD_TIME = 20.8 * 7 = 145.6ns
// 0000 0111 = 0x07
#define DEAD_TIME 0x07

#define NUM_RAW_DC_STEPS (((uint16_t) (F_SYS_CLK_HZ / ((uint32_t) PWM_FREQ_HZ * (PWM_TIM_PRESCALER + 1)))) / 2)
#define SCALING_FACTOR (MAX_DUTYCYCLE_COMMAND / NUM_RAW_DC_STEPS + 1U)
#define MAP_MAX_DUTY_TO_ARR_DUTY(dc) (dc / SCALING_FACTOR)

#define ARR_VALUE (NUM_RAW_DC_STEPS)
#define ARR_REG_VALUE (ARR_VALUE - 1)

///////////////////////////////////
//  CURRENT SENSE INSTRUMENTATION //
///////////////////////////////////

// The shunt is a single device in the DC return path below all three low-side
// FETs, so it only conducts during the active vector. Its time-average is
// D * I_phase (i.e. bus current), not phase current. See
// CURRENT_SENSING_INVESTIGATION.md for the full derivation.
//
// The firmware publishes four concurrent estimates of the same quantity so they
// can be compared at the control board:
//   filt           - ADC CH4 / PA4, post external RC LPF. Reads D * I_phase.
//   unfilt         - ADC CH3 / PA3, pre-filter, sampled at the ADC trigger.
//                    Real phase current only under CS_SYNC_SAMPLING.
//   duty corrected - filt * ARR / D_arr           (investigation Phase 0)
//   model          - (D * Vbus - Ke * w) / R_loop (investigation Phase 3)
//
// CS_SYNC_SAMPLING (investigation Phase 1) changes where in the PWM period the
// ADC fires, so it cannot run concurrently with the default configuration and is
// a compile-time gate defined per-binary in system.h. When it is on, the
// pre-filter tap lands inside the on-window and the control loop runs on it
// instead of the filtered tap.

// Below this effective on-time (in ARR counts) the duty correction divides by a
// number small enough that quantization dominates, so the result is flagged
// invalid rather than reported as a real measurement.
#ifndef CS_DUTY_CORRECTION_MIN_ARR
#define CS_DUTY_CORRECTION_MIN_ARR 10U
#endif

#ifdef CS_SYNC_SAMPLING
// Sample point within the on-window, as a fraction of the up-ramp on-segment.
// 1/2 puts it at the midpoint, which maximizes distance from both switching
// edges.
#define CS_SYNC_TRIGGER_NUM 1U
#define CS_SYNC_TRIGGER_DEN 2U
#endif

// current estimates are clamped to what the sense network can represent
#define CS_MAX_REPORTABLE_MA 9000

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void pwm6step_setup(const FixedPointS12F4_PiConstants_t *current_pi_constants);
void pwm6step_set_duty_cycle(int16_t duty_cycle);
void pwm6step_set_duty_cycle_f(float duty_cycle_pct);
void pwm6step_set_voltage(int16_t voltage_mv);
void pwm6step_set_current(int16_t current_ma);
void pwm6step_set_output_current_limit(int16_t output_current_limit_ma);

// timekeeping
bool pwm6step_1ms_flag();

// hall velocity estimate
bool pwm6step_hall_rps_estimate_valid();
int16_t pwm6step_hall_get_rps_estimate();

// external velocity estimate
//
// Publishes a velocity estimate produced outside this module (the quadrature
// encoder) so the model-based current observer can use it instead of the hall
// estimate. Units are deci-rad/s at the motor shaft, matching
// pwm6step_hall_get_rps_estimate(). Only the magnitude is consumed - rotation
// direction always comes from the hall sensors, which are the authority on
// commutation state.
void pwm6step_set_external_vel_est(int16_t vel_drads, bool valid);
// selects which estimate feeds the model observer. false (default) = hall.
void pwm6step_set_vel_est_source_external(bool use_external);
bool pwm6step_get_vel_est_source_external();

// current sense estimator outputs, all refreshed once per 1ms telemetry frame
// filtered tap (ADC CH4 / PA4): bus current, D * I_phase
const uint16_t pwm6step_get_current_est_filt_ma();
// pre-filter tap (ADC CH3 / PA3): phase current, only meaningful under CS_SYNC_SAMPLING
const uint16_t pwm6step_get_current_est_unfilt_ma();
const uint16_t pwm6step_get_current_est_duty_corrected_ma();
const int16_t pwm6step_get_current_est_model_ma();
const int16_t pwm6step_get_vel_est_used_drads();
// mean effective on-time over the frame, in ARR counts (duty = value / ARR_VALUE)
const uint16_t pwm6step_get_mean_duty_arr();
// CCM_CS_FLAG_* bits from ateam-common-packets/include/stspin_current.h
const uint8_t pwm6step_get_current_sense_flags();

// error handling and logging
const MotorErrors_t pwm6step_get_motor_errors();
const uint16_t pwm6step_get_current_measurement();
const uint16_t* pwm6step_get_current_log();
const uint16_t pwm6step_get_vbus_voltage();
const uint16_t pwm6step_get_voltage_command();
