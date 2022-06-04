/**
 * @file 6step.h
 * @author Will Stuckey
 * @brief 
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#define MAX_DUTYCYCLE_COMMAND 65535
#define MIN_DUTYCYCLE_COMMAND -(MAX_DUTYCYCLE_COMMAND)

//////////////////////
//  ERROR HANDLING  //
//////////////////////

#define HALL_DISCONNECT_ERROR_THRESHOLD 5
#define HALL_POWER_ERROR_THRESHOLD 5

#define HALL_TRANSITION_ERROR_THRESHOLD 3

////////////////////////////////
//  LOW LEVEL CONTROL PARAMS  //
////////////////////////////////

#define PWM_TIM_PRESCALER 0

// period 20000ns
// be conscious of dead time ratio
#define PWM_FREQ_HZ 50000 

// no div on CK_INT = 48MHz
// Tdts = CK_INT = 48MHz
// xxx yyyyy -> 0xx selects no multiplier
// Tdts = 20.8 ns
// yyyyy = 11110 = 30
// DEAD_TIME = 20.8 * 30 = 624ns
#define DEAD_TIME 0x1E