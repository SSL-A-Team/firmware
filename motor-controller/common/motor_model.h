
# pragma once

typedef struct MotorModel {
    float max_vel_rads;
    float enc_ticks_per_rev;
    float rated_current;
    float voltage_to_current_linear_map_m;
    float voltage_to_current_linear_map_b;
    float torque_to_current_linear_model_m;
    float torque_to_current_linear_model_b;
    float current_to_torque_linear_model_m;
    float current_to_torque_linear_model_b;
    float rads_to_dc_linear_map_m;
    float rads_to_dc_linear_map_b;
} MotorModel_t;

void mm_initialize(MotorModel_t *mm);
float mm_rads_to_dc(MotorModel_t *mm, float avel_rads);
float mm_enc_ticks_to_rev(MotorModel_t *mm, float enc_ticks);
float mm_enc_ticks_to_rad(MotorModel_t *mm, float enc_ticks);
float mm_voltage_to_current(MotorModel_t *mm, float voltage);
float mm_current_to_torque(MotorModel_t *mm, float current);
float mm_torque_to_current(MotorModel_t *mm, float torque);
float mm_pos_current_to_pos_dc(MotorModel_t *mm, float current);