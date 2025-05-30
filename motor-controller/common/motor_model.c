
#include <math.h>

#include "motor_model.h"

#define MOTOR_MINIMUM_VEL 4.0f // rad/s

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
    mm->rads_to_dc_linear_map_m = 0.0f;
    mm->rads_to_dc_linear_map_b = 0.0f;
    mm->line_resistance = 0.0f;
}

float mm_rads_to_dc(MotorModel_t *mm, float avel_rads) {
    // Need to exceed coefficient to prevent b from inverting the directions.
    // So just return 0.0 at small values
    if (avel_rads > -MOTOR_MINIMUM_VEL && avel_rads < MOTOR_MINIMUM_VEL) {
        return 0.0f;
    }

    // Linear mapping from 'motor_model.py' in hw-analysis
    float map_value = (avel_rads * mm->rads_to_dc_linear_map_m) +
        ((avel_rads < 0) ? mm->rads_to_dc_linear_map_b : -mm->rads_to_dc_linear_map_b);

    // bound DC [-1, 1]
    return fmax(fmin(2.0f * map_value, 1.0f), -1.0f);
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

float mm_pos_current_to_pos_dc(MotorModel_t *mm, float current, float vbus_voltage) {
    // I_motor = V_motor / R_motor
    // V_motor = duty_cycle * V_bus
    // duty_cycle = I_motor * (R_motor / V_bus)
    if (vbus_voltage == 0.0f) {
        return 0.0f; // avoid division by zero
    }

    // bound DC [0, 1]
    return fmax(fmin(current * (mm->line_resistance / vbus_voltage), 1.0f), 0.0f);
}