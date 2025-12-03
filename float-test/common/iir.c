/**
 * @file iir.c
 * @brief IIR filter implementation (copied from motor-controller)
 */

#include "main.h"

void iir_filter_init(IIRFilter_t *iir_filter, float alpha) {
    iir_filter->alpha = alpha;
    iir_filter->previous_value = 0.0f;
}

float iir_filter_update(IIRFilter_t *iir_filter, float cur_val) {
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    float result = iir_filter->alpha * cur_val +
                   (1.0f - iir_filter->alpha) * iir_filter->previous_value;
    iir_filter->previous_value = result;
    return result;
}
