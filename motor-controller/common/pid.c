
#include <float.h>
#include <math.h>

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
float pid_calculate(Pid_t *pid, float r, float y, float dt) {
    float err = r - y;

    float termP = err * pid->pid_constants->kP;

    pid->eI = pid->eI + (err * dt);

    if (fabs(r) < 3.0) {
        pid->eI = 0.0;
    }

    if (pid->eI > pid->pid_constants->kI_max) {
        pid->eI = pid->pid_constants->kI_max;
    } else if (pid->eI < pid->pid_constants->kI_min) {
        pid->eI = pid->pid_constants->kI_min;
    }
    float termI = pid->eI * pid->pid_constants->kI;

    float termD = ((err - pid->prev_err) / dt) * pid->pid_constants->kD; // flip err and prev_err???
    pid->prev_err = err;

    float u = r + (termP + termI + termD);
    return u;
}