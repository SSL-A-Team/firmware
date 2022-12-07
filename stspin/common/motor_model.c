
typedef struct MotorModel {
    float max_vel_rads;
    float voltage_at_no_current;
    float voltage_at_rated_current;
    float enc_ticks_per_rev;
} MotorModel_t;

float mm_rads_to_dc(MotorModel_t *mm, float rad_s) {
    return rad_s / mm->max_vel_rads;
}

float mm_voltage_to_current(MotorModel_t *mm, float voltage) {

}

float mm_current_to_torque(MotorModel_t *mm, float current) {

}

float mm_ticks_to_rads(MotorModel_t *mm, float enc_ticks, float delta_t_s) {

}