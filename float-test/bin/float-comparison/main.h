/**
 * @file main.h
 * @brief Main header for float-comparison test
 */

#pragma once

#include <stdint.h>

// Include common headers from motor-controller
// Note: These need to be copied or linked from motor-controller/common

// PID structures (from motor-controller)
typedef struct PidConstants {
    float kP;
    float kI;
    float kD;
    float kI_max;
    float kI_min;
} PidConstants_t;

typedef struct Pid {
    PidConstants_t *pid_constants;
    float eI;
    float prev_err;
} Pid_t;

void pid_constants_initialize(PidConstants_t *pid_constants);
void pid_initialize(Pid_t *pid, PidConstants_t *pid_constants);
float pid_calculate(Pid_t *pid, float r, float y, float dt);

// IIR Filter structures (from motor-controller)
typedef struct IIRFilter {
    float alpha;
    float previous_value;
} IIRFilter_t;

void iir_filter_init(IIRFilter_t *iir_filter, float alpha);
float iir_filter_update(IIRFilter_t *iir_filter, float cur_val);

// Version information
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0
