/**
 * @file fixed_point.h
 * @brief Q16.16 Fixed-point arithmetic library for performance comparison
 *
 * This library implements Q16.16 fixed-point math operations for
 * comparison against floating-point performance on embedded systems.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Q16.16 fixed point type (16 bits integer, 16 bits fractional)
typedef int32_t fixed_t;

// Conversion factor for Q16.16
#define FIXED_POINT_FRACTIONAL_BITS 16
#define FIXED_POINT_ONE (1 << FIXED_POINT_FRACTIONAL_BITS)

// Conversion macros
#define FLOAT_TO_FIXED(f) ((fixed_t)((f) * FIXED_POINT_ONE))
#define INT_TO_FIXED(i) ((fixed_t)((i) << FIXED_POINT_FRACTIONAL_BITS))
#define FIXED_TO_FLOAT(x) (((float)(x)) / FIXED_POINT_ONE)
#define FIXED_TO_INT(x) ((int)((x) >> FIXED_POINT_FRACTIONAL_BITS))

// Basic arithmetic operations
static inline fixed_t fixed_add(fixed_t a, fixed_t b) {
    return a + b;
}

static inline fixed_t fixed_sub(fixed_t a, fixed_t b) {
    return a - b;
}

static inline fixed_t fixed_mul(fixed_t a, fixed_t b) {
    return (fixed_t)(((int64_t)a * (int64_t)b) >> FIXED_POINT_FRACTIONAL_BITS);
}

static inline fixed_t fixed_div(fixed_t a, fixed_t b) {
    return (fixed_t)(((int64_t)a << FIXED_POINT_FRACTIONAL_BITS) / b);
}

static inline fixed_t fixed_abs(fixed_t x) {
    return (x < 0) ? -x : x;
}

static inline fixed_t fixed_neg(fixed_t x) {
    return -x;
}

// Comparison operations
static inline bool fixed_gt(fixed_t a, fixed_t b) {
    return a > b;
}

static inline bool fixed_lt(fixed_t a, fixed_t b) {
    return a < b;
}

static inline bool fixed_eq(fixed_t a, fixed_t b) {
    return a == b;
}

// Advanced math operations
fixed_t fixed_sqrt(fixed_t x);
fixed_t fixed_sin(fixed_t x);
fixed_t fixed_cos(fixed_t x);

// Clamping
static inline fixed_t fixed_clamp(fixed_t x, fixed_t min, fixed_t max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// Fixed-point PID constants
typedef struct FixedPidConstants {
    fixed_t kP;
    fixed_t kI;
    fixed_t kD;
    fixed_t kI_max;
    fixed_t kI_min;
} FixedPidConstants_t;

// Fixed-point PID structure
typedef struct FixedPid {
    FixedPidConstants_t *pid_constants;
    fixed_t eI;
    fixed_t prev_err;
} FixedPid_t;

void fixed_pid_initialize(FixedPid_t *pid, FixedPidConstants_t *pid_constants);
fixed_t fixed_pid_calculate(FixedPid_t *pid, fixed_t r, fixed_t y, fixed_t dt);

// Fixed-point IIR filter
typedef struct FixedIIRFilter {
    fixed_t alpha;
    fixed_t previous_value;
} FixedIIRFilter_t;

void fixed_iir_filter_init(FixedIIRFilter_t *iir_filter, fixed_t alpha);
fixed_t fixed_iir_filter_update(FixedIIRFilter_t *iir_filter, fixed_t cur_val);
