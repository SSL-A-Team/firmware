/**
 * @file system.h
 * @author Will Stuckey
 * @brief
 * @version 0.1
 * @date 2022-05-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

// if the system has an encoder
#define HAS_ENCODER
// if the system is using the external encoder connector rather than the
// internal encoder
// #define HAS_EXTERNAL_ENCODER

///////////////////
//  UART PARAMS  //
///////////////////

#define UART_ENABLED
// CcmResponse is 84 bytes as of the current sense estimator telemetry
// additions. Must stay >= sizeof(CcmResponse).
#define IOQ_BUF_LENGTH 84
#define IOQ_BUF_DEPTH 4

//////////////////
//  ADC Config  //
//////////////////

#define ADC_MODE CS_MODE_DMA
// CH3 = PA3 current sense pre-filter, CH4 = PA4 current sense post external RC
// LPF, CH9 = bus voltage divider. Order matters: the ADC converts in ascending
// channel index, and ADC_Result_t's field order mirrors that.
#define ADC_NUM_CHANNELS 3
#define ADC_DMA_NUM_TRANSFERS 3
#define ADC_CH_MASK (ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL9)
#define ADC_SR_MASK (ADC_SMPR_SMP_0)

////////////////////////////////
//  CURRENT SENSE SAMPLING MODE   //
////////////////////////////////////

// Investigation Phase 1 (see CURRENT_SENSING_INVESTIGATION.md). Uncomment to
// move the ADC trigger inside the PWM on-window so the shunt is sampled while
// it is actually conducting, instead of reading the analog filter's time
// average (which is bus current, D * I_phase).
//
// This changes what the ADC physically samples, so unlike the duty correction
// and the model observer it cannot run concurrently with the default
// configuration - it is a compile-time choice. Enabling it:
//   - makes TIM1 CCR4 track duty                    (6step_current.c)
//   - drops TIM1 CMS to up-count-only flags         (6step_current.c)
//   - preloads CCR4                                 (6step_current.c)
//   - shortens the ADC aperture to 125ns            (current_sensing.c)
// It also requires the pre-filter ADC channel; with the ~2 kHz analog LPF in
// the path the trigger placement is irrelevant. Telemetry reports the active
// mode via CCM_CS_FLAG_SYNC_SAMPLING.
//
// Not viable below roughly 10 ARR counts of on-time (~1.7% duty): the trigger
// sits too close to the switching edge for ringing to settle.
#define CS_SYNC_SAMPLING

////////////////////
//  TIME KEEPING  //
////////////////////

#define MS_PER_S 1000

#define F_SYS_CLK_HZ 48000000UL
#define SYSTICK_PER_S 1000

#define VELOCITY_LOOP_RATE_MS 1
#define VELOCITY_LOOP_RATE_S ((float) VELOCITY_LOOP_RATE_MS / (float) MS_PER_S)
#define TORQUE_LOOP_RATE_MS 1
#define TORQUE_LOOP_RATE_S ((float) TORQUE_LOOP_RATE_MS / (float) MS_PER_S)
#define TELEMETRY_LOOP_RATE_MS 1

#define MOTOR_MAXIMUM_ACCEL 6000 // rad/s^2

// #define MOTOR_MAXIMUM_RAD_S 550.825911929f

#define COMP_MODE