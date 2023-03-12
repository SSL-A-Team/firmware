
#pragma once

typedef struct IIRFilter {
    float alpha;
    float previous_value;
} IIRFilter_t;

float iir_filter_alpha_from_Tf(float Tf, float Ts);

void iir_filter_init(IIRFilter_t *iir_filter, float alpha);

float iir_filter_update(IIRFilter_t *iir_filter, float cur_val);