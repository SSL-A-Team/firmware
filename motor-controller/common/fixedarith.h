// uncomment to enable rounding numbers
// NOTE: feature incomplete
// #define FIXEDPOINT_ARITH_ROUND

#include <stdint.h>

#pragma once

typedef struct FixedPointConfig {
    uint8_t total_bits;
    uint8_t int_bits;
    uint8_t frac_bits;
} FixedPointConfig_t;

typedef int32_t Int16FixedPoint_t;
typedef int32_t Int16OperationContainer_t;
typedef uint16_t Uint16FixedPoint_t;
typedef uint32_t Uint16OperationContainer_t;
typedef int32_t Int32FixedPoint_t;
typedef int64_t Int32OperationContainer_t;
typedef uint32_t Uint32FixedPoint_t;
typedef uint64_t Uint32OperationContainer_t;

extern const Int16FixedPoint_t FixedPointS12_MAX;
extern const Int16FixedPoint_t FixedPointS12_MIN;

extern const FixedPointConfig_t FixedU12F4Config;
extern const FixedPointConfig_t FixedS12F4Config;