#pragma once

#include <stdint.h>

#include "fixedarith.h"

/**
 * Create an unsigned fixed point number with at least 16 bits of storage.
 */
Uint16FixedPoint_t uint16fixed_set_from_int(uint16_t val, const FixedPointConfig_t *new_config) {
    return (val << new_config->frac_bits);
}

/**
 * Create an unsigned fixed point number with at least 16 bits of storage from another unsigned fixed point number.
 */
Uint16FixedPoint_t uint16fixed_set_from_uint16fixed(Uint16FixedPoint_t val, const FixedPointConfig_t *val_config, const FixedPointConfig_t *new_config) {
    if (val_config->frac_bits == new_config->frac_bits) {
        return val;
    } else if (val_config->frac_bits > new_config->frac_bits) {
        return val >> (val_config->frac_bits - new_config->frac_bits);
    } else {
        return val << (new_config->frac_bits - val_config->frac_bits);
    }
}

uint16_t uint16fixed_to_int(Uint16FixedPoint_t val, const FixedPointConfig_t *val_config) {
    return val >> val_config->frac_bits;
}

/**
 * Add two fixed point numbers, frac bits must be the same
 */
Uint16FixedPoint_t* uint16fixed_add(Uint16FixedPoint_t lhs, Uint16FixedPoint_t rhs) {
    return lhs + rhs;
}

Uint16FixedPoint_t* uint16fixed_sub(Uint16FixedPoint_t lhs, Uint16FixedPoint_t rhs) {
    return lhs - rhs;
}

Uint16FixedPoint_t* uint16fixed_mul(Uint16FixedPoint_t lhs, Uint16FixedPoint_t rhs, const FixedPointConfig_t *config) {
    Uint16OperationContainer_t ires = (Uint16OperationContainer_t) lhs * (Uint16OperationContainer_t) rhs;

    // (1 << config->frac_bits) term provides rounding
    #ifdef FIXEDPOINT_ARITH_ROUND
    return (Uint16FixedPoint_t) ((ires + (1 << config->frac_bits)) >> config->frac_bits);
    #else
    return (Uint16FixedPoint_t) (ires >> config->frac_bits);
    #endif
}

Uint16FixedPoint_t* uint16fixed_div(Uint16FixedPoint_t numerator, Uint16FixedPoint_t denominator, const FixedPointConfig_t *config) {
    Uint16OperationContainer_t tdiv = (Uint16OperationContainer_t) numerator * (Uint16OperationContainer_t) (1 << config->frac_bits);
    return (Uint16FixedPoint_t) (tdiv / denominator);
}