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

#define HAS_ENCODER

///////////////////
//  UART PARAMS  //
///////////////////

#define UART_ENABLED
#define IOQ_BUF_LENGTH 80
#define IOQ_BUF_DEPTH 4

//////////////////
//  ADC Config  //
//////////////////

#define ADC_MODE CS_MODE_DMA
#define ADC_NUM_CHANNELS 2
// #define ADC_CH_MASK (ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17)
#define ADC_CH_MASK (ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL9)
// #define ADC_CH_MASK (ADC_CHSELR_CHSEL4)
// #define ADC_SR_MASK (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1)
// #define ADC_SR_MASK (ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2)
#define ADC_SR_MASK (ADC_SMPR_SMP_0)
// #define ADC_SR_MASK 0

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