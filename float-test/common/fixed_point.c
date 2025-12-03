/**
 * @file fixed_point.c
 * @brief Q16.16 Fixed-point arithmetic library implementation
 */

#include "fixed_point.h"

// Square root using Newton's method
fixed_t fixed_sqrt(fixed_t x) {
    if (x <= 0) return 0;

    fixed_t guess = x >> 1; // Initial guess
    if (guess == 0) guess = 1;

    // Newton's method: x_new = (x_old + n/x_old) / 2
    for (int i = 0; i < 10; i++) {
        fixed_t new_guess = (guess + fixed_div(x, guess)) >> 1;
        if (fixed_abs(new_guess - guess) < 10) break;
        guess = new_guess;
    }

    return guess;
}

// Sine using Taylor series approximation
// sin(x) ≈ x - x³/3! + x⁵/5! - x⁷/7!
fixed_t fixed_sin(fixed_t x) {
    // Normalize x to [-π, π]
    const fixed_t PI = FLOAT_TO_FIXED(3.14159265359f);
    const fixed_t TWO_PI = FLOAT_TO_FIXED(6.28318530718f);

    while (x > PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;

    // Taylor series
    fixed_t x2 = fixed_mul(x, x);
    fixed_t x3 = fixed_mul(x2, x);
    fixed_t x5 = fixed_mul(x3, x2);
    fixed_t x7 = fixed_mul(x5, x2);

    fixed_t result = x;
    result -= fixed_div(x3, INT_TO_FIXED(6));      // -x³/3!
    result += fixed_div(x5, INT_TO_FIXED(120));    // +x⁵/5!
    result -= fixed_div(x7, INT_TO_FIXED(5040));   // -x⁷/7!

    return result;
}

// Cosine using sin identity: cos(x) = sin(x + π/2)
fixed_t fixed_cos(fixed_t x) {
    const fixed_t PI_2 = FLOAT_TO_FIXED(1.57079632679f);
    return fixed_sin(fixed_add(x, PI_2));
}

// Fixed-point PID initialization
void fixed_pid_initialize(FixedPid_t *pid, FixedPidConstants_t *pid_constants) {
    pid->pid_constants = pid_constants;
    pid->eI = 0;
    pid->prev_err = 0;
}

// Fixed-point PID calculation
fixed_t fixed_pid_calculate(FixedPid_t *pid, fixed_t r, fixed_t y, fixed_t dt) {
    // Calculate error
    fixed_t error = fixed_sub(r, y);

    // Proportional term
    fixed_t P = fixed_mul(pid->pid_constants->kP, error);

    // Integral term
    fixed_t eI_update = fixed_mul(error, dt);
    pid->eI = fixed_add(pid->eI, eI_update);

    // Clamp integral
    if (pid->eI > pid->pid_constants->kI_max) {
        pid->eI = pid->pid_constants->kI_max;
    } else if (pid->eI < pid->pid_constants->kI_min) {
        pid->eI = pid->pid_constants->kI_min;
    }

    fixed_t I = fixed_mul(pid->pid_constants->kI, pid->eI);

    // Derivative term
    fixed_t error_diff = fixed_sub(error, pid->prev_err);
    fixed_t D = fixed_mul(pid->pid_constants->kD, fixed_div(error_diff, dt));

    pid->prev_err = error;

    // Combine terms
    fixed_t output = fixed_add(fixed_add(P, I), D);

    return output;
}

// Fixed-point IIR filter initialization
void fixed_iir_filter_init(FixedIIRFilter_t *iir_filter, fixed_t alpha) {
    iir_filter->alpha = alpha;
    iir_filter->previous_value = 0;
}

// Fixed-point IIR filter update
fixed_t fixed_iir_filter_update(FixedIIRFilter_t *iir_filter, fixed_t cur_val) {
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    fixed_t one_minus_alpha = fixed_sub(FIXED_POINT_ONE, iir_filter->alpha);
    fixed_t term1 = fixed_mul(iir_filter->alpha, cur_val);
    fixed_t term2 = fixed_mul(one_minus_alpha, iir_filter->previous_value);

    fixed_t result = fixed_add(term1, term2);
    iir_filter->previous_value = result;

    return result;
}
