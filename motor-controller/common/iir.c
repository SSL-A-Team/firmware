#include "iir.h"
#include "math_util.h"

float iir_filter_alpha_from_cutoff_rads(const float Fcutoff_rads, const float Ts) {
    float sample_rate = 1.0f / Ts;
    float alpha = approx3_expf(-1.0f * (Fcutoff_rads / sample_rate));

    return alpha;
}

float iir_filter_alpha_from_cutoff_hz(const float bw_hz, const float Ts) {
    return iir_filter_alpha_from_cutoff_rads(bw_hz * 2.0f * F_PI, Ts);
}

void iir_filter_init(IIRFilter_t *iir_filter, float alpha) {
    iir_filter->alpha = alpha;
    iir_filter->previous_value = 0.0f;
}

float iir_filter_update(IIRFilter_t *iir_filter, float cur_val) {
    float Vf = (iir_filter->alpha * iir_filter->previous_value) + ((1.0f - iir_filter->alpha) * cur_val);
    iir_filter->previous_value = Vf;
    return Vf;
}