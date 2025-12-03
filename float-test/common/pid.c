/**
 * @file pid.c
 * @brief PID controller implementation (copied from motor-controller)
 */

#include "main.h"

void pid_constants_initialize(PidConstants_t *pid_constants) {
    pid_constants->kP = 0.0f;
    pid_constants->kI = 0.0f;
    pid_constants->kD = 0.0f;
    pid_constants->kI_max = 0.0f;
    pid_constants->kI_min = 0.0f;
}

void pid_initialize(Pid_t *pid, PidConstants_t *pid_constants) {
    pid->pid_constants = pid_constants;
    pid->eI = 0.0f;
    pid->prev_err = 0.0f;
}

float pid_calculate(Pid_t *pid, float r, float y, float dt) {
    // Calculate error
    float error = r - y;

    // Proportional term
    float P = pid->pid_constants->kP * error;

    // Integral term
    pid->eI += error * dt;

    // Clamp integral
    if (pid->eI > pid->pid_constants->kI_max) {
        pid->eI = pid->pid_constants->kI_max;
    } else if (pid->eI < pid->pid_constants->kI_min) {
        pid->eI = pid->pid_constants->kI_min;
    }

    float I = pid->pid_constants->kI * pid->eI;

    // Derivative term
    float error_diff = error - pid->prev_err;
    float D = pid->pid_constants->kD * (error_diff / dt);

    pid->prev_err = error;

    // Combine terms
    return P + I + D;
}
