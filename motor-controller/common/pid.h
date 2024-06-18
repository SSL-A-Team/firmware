
#pragma once

typedef struct PidConstants {
    float kP;
    float kI;
    float kD;

    float kI_max;
    float kI_min;
} PidConstants_t;

typedef struct Pid {
    PidConstants_t *pid_constants;

    //float prev_u;

    float eI;
    float prev_err;
} Pid_t;

void pid_constants_initialize(PidConstants_t *pid_constants);

void pid_initialize(Pid_t *pid, PidConstants_t *pid_constants);

float pid_calculate(Pid_t *pid, float r, float y, float dt);