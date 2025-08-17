
#pragma once

#include "stddef.h"
#include "stdbool.h"

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

typedef enum GainScheduledPidResult {
    PID_INIT_OK,
    PID_NUM_TUNING_POINTS_TOO_LOW,
    PID_CONSTANTS_BAD_LENGTH,
    GAIN_SCHEDULE_BAD_LENGTH,
    GAIN_SCHEDULE_NON_MONOTONIC,
    GAIN_SCHEDULE_PID_CONSTANTS_LENS_NOT_EQUAL,
    HYST_PCT_INVALID,
} GainScheduledPidResult_t;

typedef struct GainScheduledPid {
    size_t num_gain_stages;
    PidConstants_t *pid_constants;
    float *gain_schedule;
    size_t cur_gain_stage_ind;
    PidConstants_t cur_pid_constants;
    bool gain_schedule_abs;
    float hyst;

    float eI;
    float prev_err;
} GainScheduledPid_t;

GainScheduledPidResult_t gspid_initialize(GainScheduledPid_t *pid, size_t num_gain_stages, PidConstants_t *pid_constants, float *gain_schedule, float hyst_pct, bool gain_schedule_abs);
float gspid_calculate(GainScheduledPid_t *pid, float r, float y, float dt);
size_t gspid_get_cur_gain_stage_index(GainScheduledPid_t *pid);