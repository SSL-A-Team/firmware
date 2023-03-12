
#include <math.h>

#include "motor_model.h"

void mm_initialize(MotorModel_t *mm) {
    mm->max_vel_rads = 0.0f;
    mm->enc_ticks_per_rev = 0.0f;
    mm->rated_current = 0.0f;
    mm->voltage_to_current_linear_map_m = 0.0f;
    mm->voltage_to_current_linear_map_b = 0.0f;
    mm->torque_to_current_linear_model_m = 0.0f;
    mm->torque_to_current_linear_model_b = 0.0f;
    mm->current_to_torque_linear_model_m = 0.0f;
    mm->current_to_torque_linear_model_b = 0.0f;
}

float mm_rads_to_dc(MotorModel_t *mm, float avel_rads) {
    // bound DC [-1, 1]
    return fmax(fmin(avel_rads / mm->max_vel_rads, 1.0f), -1.0f);
}

float mm_enc_ticks_to_rev(MotorModel_t *mm, float enc_ticks) {
    return (enc_ticks / mm->enc_ticks_per_rev);
}

float mm_enc_ticks_to_rad(MotorModel_t *mm, float enc_ticks) {
    return mm_enc_ticks_to_rev(mm, enc_ticks) * 2.0f * (float) M_PI;
}

float mm_voltage_to_current(MotorModel_t *mm, float voltage) {
    // bound I [0, inf)
    // if voltage dips below min value due to noise, don't report a negative current. That makes no sense.
    return fmax(mm->voltage_to_current_linear_map_m * voltage + mm->voltage_to_current_linear_map_b, 0.0f);
}

float mm_current_to_torque(MotorModel_t *mm, float current) {
    return fmax(mm->current_to_torque_linear_model_m * current + mm->current_to_torque_linear_model_b, 0.0f);
}

float mm_torque_to_current(MotorModel_t *mm, float torque) {
    return fmax(mm->torque_to_current_linear_model_m * torque + mm->torque_to_current_linear_model_b, 0.0f);
}

float mm_pos_current_to_pos_dc(MotorModel_t *mm, float current) {
    return fmax(fmin(current / mm->rated_current, 1.0f), 0.0f);
}