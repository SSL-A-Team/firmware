
#include "iir.h"

float iir_filter_alpha_from_Tf(float Tf, float Ts) {
    return (Tf / (Tf + Ts));
}

void iir_filter_init(IIRFilter_t *iir_filter, float alpha) {
    iir_filter->alpha = alpha;
    iir_filter->previous_value = 0.0f;
}

float iir_filter_update(IIRFilter_t *iir_filter, float cur_val) {
    float Vf = (iir_filter->alpha * iir_filter->previous_value) + ((1.0f - iir_filter->alpha) * cur_val);
    iir_filter->previous_value = cur_val;
    return Vf;
}