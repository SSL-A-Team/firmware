#include "math_util.h"

float approx2_expf(const float x) {
    return 1.0f + x + (x * x) / 2.0f;
}

float approx3_expf(const float x) {
    return 1.0f + (x) + ((x * x) / 2.0f) + ((x * x * x) / 6.0f);
}