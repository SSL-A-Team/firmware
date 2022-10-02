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

///////////////////
//  UART PARAMS  //
///////////////////

//#define UART_ENABLED
#define IOQ_BUF_LENGTH 64
#define IOQ_BUF_DEPTH 4

//////////////////
//  ADC Config  //
//////////////////

#define ADC_MODE CS_MODE_TIMER_DMA
#define ADC_NUM_CHANNELS 5
#define ADC_CH_MASK (ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17)
// #define ADC_SR_MASK (ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2)
#define ADC_SR_MASK (ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2)
// #define ADC_SR_MASK (ADC_SMPR_SMP_0)


////////////////////
//  TIME KEEPING  //
////////////////////

#define MS_PER_S 1000

#define F_SYS_CLK_HZ 48000000UL
#define SYSTICK_PER_S 1000

#define LOOP_RATE_MS 1
#define LOOP_RATE_S ((float) LOOP_RATE_MS / (float) MS_PER_S)

////////////////////////
//  FILTERING/TUNING  //
////////////////////////

#define ADC_IIR_TF_MS 0.20f

#define MOTOR_MAXIMUM_RAD_S 550.825911929f