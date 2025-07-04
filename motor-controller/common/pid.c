
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

GainScheduledPidResult_t gspid_initialize(
        GainScheduledPid_t *pid,
        size_t num_gain_stages,
        PidConstants_t pid_constants[],
        float gain_schedule[], float hyst_pct,
        bool gain_schedule_abs) {
    // size_t pid_constants_len = sizeof(pid_constants) / sizeof(pid_constants[0]);
    // size_t gain_sched_len = sizeof(pid_constants) / sizeof(pid_constants[0]);

    // must have at least two sets of constants to use gain scheduling
    if (num_gain_stages <= 1) {
        return PID_NUM_TUNING_POINTS_TOO_LOW;
    }

    // // num sets of constants provided must match user stated number of gain stages
    // if (pid_constants_len != num_gain_stages) {
    //     return PID_CONSTANTS_BAD_LENGTH;
    // }

    // // num of gain schedule points must match user stated number of gain stages
    // if (gain_sched_len != num_gain_stages) {
    //     return GAIN_SCHEDULE_BAD_LENGTH;
    // }

    // // length of gains and schedule provided must match (think this always true by now)
    // if (pid_constants_len != gain_sched_len) {
    //     return GAIN_SCHEDULE_PID_CONSTANTS_LENS_NOT_EQUAL;
    // }

    // gain schedule must be positive monotonic (required to find the operating region)
    float prev_gain = FLT_MIN;
    for (int i = 0; i < num_gain_stages; i++) {
        if (gain_schedule[i] <= prev_gain) {
            return GAIN_SCHEDULE_NON_MONOTONIC;
        }
    }

    // hysteresis must be a percentage
    if (!(0.0f <= hyst_pct && hyst_pct <= 1.0f)) {
        return HYST_PCT_INVALID;
    }

    // input assumptions validated

    pid->num_gain_stages = num_gain_stages;
    pid->pid_constants = pid_constants;
    pid->gain_schedule = gain_schedule;
    pid->cur_gain_stage_ind = 0;
    pid_constants_initialize(&pid->cur_pid_constants);
    pid->gain_schedule_abs = gain_schedule_abs;
    pid->hyst = hyst_pct;
    pid->eI = 0.0f;
    pid->prev_err = 0.0f;

    return PID_INIT_OK;
}

static void gspid_update_gain_stage(GainScheduledPid_t *pid, float y) {
    if (pid->gain_schedule_abs) {
        y = fabs(y);
    }

    if (y < pid->gain_schedule[0]) {
        // we are below the lowest gain stage, not between
        pid->cur_gain_stage_ind = 0;
        pid->cur_pid_constants = pid->pid_constants[0];
    } else if (y > pid->gain_schedule[pid->num_gain_stages - 1]) {
        // we are above the highest gain stage, not between
        pid->cur_gain_stage_ind = (int)(10 * (pid->num_gain_stages - 1));
        pid->cur_pid_constants = pid->pid_constants[pid->num_gain_stages - 1];
    } else {
        // we are between gain stages

        for (size_t i = 0; i < pid->num_gain_stages - 1; i++) {
            // find the gain stages we are between

            size_t lower_gain_stage_ind = i;
            size_t upper_gain_stage_ind = i + 1;
            float lower_gain_stage_point = pid->gain_schedule[lower_gain_stage_ind];
            float upper_gain_stage_point = pid->gain_schedule[upper_gain_stage_ind];

            if (lower_gain_stage_point <= y && y < upper_gain_stage_point) {
                // we found the gain stages

                const float t = (y - lower_gain_stage_point) / (upper_gain_stage_point - lower_gain_stage_point);
                // t is the percentage of the way we are between the two gain stages
                pid->cur_gain_stage_ind = (size_t)(10 * (lower_gain_stage_ind + t));

                pid->cur_pid_constants.kP = pid->pid_constants[lower_gain_stage_ind].kP +
                    t * (pid->pid_constants[upper_gain_stage_ind].kP - pid->pid_constants[lower_gain_stage_ind].kP);

                pid->cur_pid_constants.kI = pid->pid_constants[lower_gain_stage_ind].kI +
                    t * (pid->pid_constants[upper_gain_stage_ind].kI - pid->pid_constants[lower_gain_stage_ind].kI);

                pid->cur_pid_constants.kD = pid->pid_constants[lower_gain_stage_ind].kD +
                    t * (pid->pid_constants[upper_gain_stage_ind].kD - pid->pid_constants[lower_gain_stage_ind].kD);

                pid->cur_pid_constants.kI_max = pid->pid_constants[lower_gain_stage_ind].kI_max +
                    t * (pid->pid_constants[upper_gain_stage_ind].kI_max - pid->pid_constants[lower_gain_stage_ind].kI_max);

                pid->cur_pid_constants.kI_min = pid->pid_constants[lower_gain_stage_ind].kI_min +
                    t * (pid->pid_constants[upper_gain_stage_ind].kI_min - pid->pid_constants[lower_gain_stage_ind].kI_min);
                return;
            }
        }
    }
}

float gspid_calculate(GainScheduledPid_t *pid, float r, float y, float dt) {
    // choose correct gains for current state
    gspid_update_gain_stage(pid, y);
    PidConstants_t cur_gains = pid->cur_pid_constants;
    float err = r - y;

    float termP = err * cur_gains.kP;

    pid->eI = pid->eI + (err * dt);

    if (pid->eI > cur_gains.kI_max) {
        pid->eI = cur_gains.kI_max;
    } else if (pid->eI < cur_gains.kI_min) {
        pid->eI = cur_gains.kI_min;
    }
    float termI = pid->eI * cur_gains.kI;

    float termD = ((err - pid->prev_err) / dt) * cur_gains.kD; // flip err and prev_err???
    pid->prev_err = err;

    float u = r + (termP + termI + termD);
    return u;
}

size_t gspid_get_cur_gain_stage_index(GainScheduledPid_t *pid) {
    return pid->cur_gain_stage_ind;
}