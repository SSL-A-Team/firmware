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

#define MAX_DUTYCYCLE_COMMAND 65535U
#define MIN_DUTYCYCLE_COMMAND -(MAX_DUTYCYCLE_COMMAND)

// period 20833ns
// be conscious of dead time ratio
#define PWM_TIM_PRESCALER 0
#define PWM_FREQ_HZ 48000

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

#define NUM_RAW_DC_STEPS (uint16_t) (F_SYS_CLK_HZ / ((uint32_t) PWM_FREQ_HZ * (PWM_TIM_PRESCALER + 1)))
#define MINIMUM_EFFECTIVE_DUTY_CYCLE_RAW 10U
#define SCALING_FACTOR ((MAX_DUTYCYCLE_COMMAND / (NUM_RAW_DC_STEPS - MINIMUM_EFFECTIVE_DUTY_CYCLE_RAW)) + 1U)
#define MAP_UINT16_TO_RAW_DC(dc) (dc > 0 ? ((dc / SCALING_FACTOR) + MINIMUM_EFFECTIVE_DUTY_CYCLE_RAW) : 0U)

// #define FORCE_COMUTATION_TRANSITION_DELAY

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void pwm6step_setup();
void pwm6step_set_duty_cycle_f(float duty_cycle_pct);
void pwm6step_set_duty_cycle(int32_t duty_cycle);
void pwm6step_brake(uint16_t braking_force);
void pwm6step_stop();
void pwm6step_estop();
void pwm6step_invert_direction(bool invert);
bool pwm6step_is_direction_inverted();
const MotorErrors_t pwm6step_get_motor_errors();
bool pwm6step_hall_rps_estimate_valid();
int pwm6step_hall_get_rps_estimate();