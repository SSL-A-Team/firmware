#include <math.h>

#include "iir.h"

const float PI = 3.1415927f;

float iir_filter_alpha_from_bw_rads(const float bw_rads, const float Ts) {
    float sample_rate = 1.0f / Ts;
    float wc = bw_rads / sample_rate; // wc is in radians/sample

    float alpha = (cosf(wc) - sinf(wc) + 1.0f) / (cosf(wc) + sinf(wc) + 1.0f);

    return alpha;
}

float iir_filter_alpha_from_bw_hz(const float bw_hz, const float Ts) {
    return iir_filter_alpha_from_bw_rads(bw_hz * 2.0f * PI, Ts);
}

float iir_filter_alpha_from_Tf(float Tf, float Ts) {
    return (Tf / (Tf + Ts));
}

void iir_filter_init(IIRFilter_t *iir_filter, float alpha) {
    iir_filter->alpha = alpha;
    iir_filter->previous_value = 0.0f;
}

float iir_filter_update(IIRFilter_t *iir_filter, float cur_val) {
    float Vf = (iir_filter->alpha * cur_val) + ((1.0f - iir_filter->alpha) * iir_filter->previous_value);
    iir_filter->previous_value = Vf;
    return Vf;
}