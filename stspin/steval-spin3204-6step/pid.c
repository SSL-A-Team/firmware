
#include <float.h>

#include "pid.h"

void pid_constants_initialize(PidConstants_t *pid_constants) {
    pid_constants->kP = 0.0f;
    pid_constants->kI = 0.0f;
    pid_constants->kD = 0.0f;

    pid_constants->kI_max = FLT_MAX;
    pid_constants->kI_min = FLT_MIN;
}

void pid_initialize(Pid_t *pid, PidConstants_t *pid_constants) {
    pid->pid_constants = pid_constants;

    //pid->prev_u = 0.0f;

    pid->eI = 0.0f;
    pid->prev_err = 0.0f;
}

// this graphic might be helpful
// https://upload.wikimedia.org/wikipedia/commons/4/43/PID_en.svg
float pid_calculate(Pid_t *pid, float r, float y) {
    float err = r - y;
    
    float termP = err * pid->pid_constants->kP;

    float termI = pid->eI + (err * pid->pid_constants->kI);
    if (termI > pid->pid_constants->kI_max) {
        termI = pid->pid_constants->kI_max;
    } else if (termI < pid->pid_constants->kI_min) {
        termI = pid->pid_constants->kI_min;
    }
    pid->eI = termI;

    float termD = (err - pid->prev_err) * pid->pid_constants->kD; // flip err and prev_err???
    pid->prev_err = err;

    float u = r + (termP + termI + termD);
    //pid->prev_u = u;
    return u;
}