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

#include <stdbool.h>
#include <stdint.h>

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
#define PWM_FREQ_HZ 48000

// no div on CK_INT = 48MHz
// Tdts = CK_INT = 48MHz
// xxx yyyyy -> 0xx selects no multiplier
// Tdts = 20.8 ns
// yyyyy = 11110 = 30
// DEAD_TIME = 20.8 * 30 = 624ns
#define DEAD_TIME 0x1E

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void pwm6step_setup();
void pwm6step_set_duty_cycle(int32_t duty_cycle);
void pwm6step_stop();
void pwm6step_invert_direction(bool invert);
bool pwm6step_is_direction_inverted();
bool pwm6step_hall_rps_estimate_valid();
int pwm6step_hall_get_rps_estimate();