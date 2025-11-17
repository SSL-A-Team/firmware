// uncomment to enable rounding numbers
// NOTE: feature incomplete
// #define FIXEDPOINT_ARITH_ROUND

#include <stdint.h>

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

const Int16FixedPoint_t FixedPointS12_MAX = 4095;
const Int16FixedPoint_t FixedPointS12_MIN = -4096;

const FixedPointConfig_t FixedU12F4Config = {
    .total_bits = 16,
    .int_bits = 12,
    .frac_bits = 4,
};

const FixedPointConfig_t FixedS12F4Config = {
    .total_bits = 16,
    .int_bits = 12,
    .frac_bits = 4,
};