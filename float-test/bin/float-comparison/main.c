/**
 * @file main.c
 * @brief Float vs Fixed-Point Performance Comparison Test
 *
 * This program evaluates the performance differences between floating-point
 * and fixed-point arithmetic on the STM32G071RB microcontroller (Nucleo-G071RB).
 *
 * Tests include:
 * - Basic arithmetic operations
 * - PID controller calculations
 * - IIR filter operations
 * - Trigonometric functions
 * - Motor control simulation
 */

#include <stm32g071xx.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include "fixed_point.h"
#include "main.h"
#include "setup.h"
#include "time.h"

// Enable semihosting for printf output via OpenOCD
extern void initialise_monitor_handles(void);

// Test configuration
#define NUM_ITERATIONS 100  // Reduced for faster testing
#define TEST_ARRAY_SIZE 100

// Result structure for timing measurements
typedef struct TestResults {
    uint32_t float_cycles;
    uint32_t fixed_cycles;
    float float_result;
    float fixed_result;
    float error_pct;
} TestResults_t;

// Global test arrays
static float float_input[TEST_ARRAY_SIZE];
static fixed_t fixed_input[TEST_ARRAY_SIZE];

// Cycle counter - using SysTick on Cortex-M0 with overflow tracking
static volatile uint32_t overflow_count = 0;

/**
 * @brief SysTick interrupt handler - called on overflow
 */
void SysTick_Handler(void) {
    overflow_count++;
}

/**
 * @brief Initialize SysTick for cycle counting
 * Note: Cortex-M0 doesn't have DWT, so we use SysTick with overflow tracking
 */
static void dwt_init(void) {
    // Configure SysTick to run at CPU frequency with interrupts
    SysTick->LOAD = 0xFFFFFF; // Maximum reload value (24-bit)
    SysTick->VAL = 0;
    overflow_count = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |  // Enable interrupt
                    SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief Get current CPU cycle count with overflow tracking
 */
static inline uint32_t dwt_get_cycles(void) {
    uint32_t overflow1, overflow2, val;

    // Read overflow count twice to detect if overflow happened during read
    do {
        overflow1 = overflow_count;
        val = SysTick->VAL;
        overflow2 = overflow_count;
    } while (overflow1 != overflow2);

    // SysTick counts down, so we invert it and add overflows
    return (0xFFFFFF - val) + (overflow1 * 0x1000000);
}

/**
 * @brief Initialize test data arrays
 */
static void init_test_data(void) {
    for (int i = 0; i < TEST_ARRAY_SIZE; i++) {
        float val = (float)i * 0.1f - 5.0f;
        float_input[i] = val;
        fixed_input[i] = FLOAT_TO_FIXED(val);
    }
}

/**
 * @brief Test 1: Basic Arithmetic Operations
 */
static void test_basic_arithmetic(TestResults_t *result) {
    uint32_t start, end;
    volatile float float_result = 0.0f;
    volatile fixed_t fixed_result = 0;

    // Floating-point test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        for (int i = 0; i < TEST_ARRAY_SIZE - 1; i++) {
            float a = float_input[i];
            float b = float_input[i + 1];
            float_result = (a * b) + (a / b) - (a * 2.5f);
        }
    }
    end = dwt_get_cycles();
    result->float_cycles = end - start;
    result->float_result = float_result;

    // Fixed-point test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        for (int i = 0; i < TEST_ARRAY_SIZE - 1; i++) {
            fixed_t a = fixed_input[i];
            fixed_t b = fixed_input[i + 1];
            fixed_result = fixed_add(fixed_mul(a, b),
                          fixed_sub(fixed_div(a, b),
                          fixed_mul(a, FLOAT_TO_FIXED(2.5f))));
        }
    }
    end = dwt_get_cycles();
    result->fixed_cycles = end - start;
    result->fixed_result = FIXED_TO_FLOAT(fixed_result);

    // Calculate error
    if (fabsf(result->float_result) > 1e-6f) {
        result->error_pct = 100.0f * fabsf(result->float_result - result->fixed_result) / fabsf(result->float_result);
    } else {
        result->error_pct = 0.0f;
    }
}

/**
 * @brief Test 2: PID Controller
 */
static void test_pid_controller(TestResults_t *result) {
    uint32_t start, end;

    // Float PID setup
    PidConstants_t float_pid_constants = {
        .kP = 1.5f,
        .kI = 0.5f,
        .kD = 0.1f,
        .kI_max = 10.0f,
        .kI_min = -10.0f
    };
    Pid_t float_pid;
    pid_initialize(&float_pid, &float_pid_constants);

    // Fixed PID setup
    FixedPidConstants_t fixed_pid_constants = {
        .kP = FLOAT_TO_FIXED(1.5f),
        .kI = FLOAT_TO_FIXED(0.5f),
        .kD = FLOAT_TO_FIXED(0.1f),
        .kI_max = FLOAT_TO_FIXED(10.0f),
        .kI_min = FLOAT_TO_FIXED(-10.0f)
    };
    FixedPid_t fixed_pid;
    fixed_pid_initialize(&fixed_pid, &fixed_pid_constants);

    volatile float float_result = 0.0f;
    volatile fixed_t fixed_result = 0;
    float dt = 0.001f; // 1ms timestep
    fixed_t dt_fixed = FLOAT_TO_FIXED(dt);

    // Floating-point PID test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        float setpoint = 10.0f;
        float measurement = float_input[i % TEST_ARRAY_SIZE];
        float_result = pid_calculate(&float_pid, setpoint, measurement, dt);
    }
    end = dwt_get_cycles();
    result->float_cycles = end - start;
    result->float_result = float_result;

    // Fixed-point PID test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        fixed_t setpoint = FLOAT_TO_FIXED(10.0f);
        fixed_t measurement = fixed_input[i % TEST_ARRAY_SIZE];
        fixed_result = fixed_pid_calculate(&fixed_pid, setpoint, measurement, dt_fixed);
    }
    end = dwt_get_cycles();
    result->fixed_cycles = end - start;
    result->fixed_result = FIXED_TO_FLOAT(fixed_result);

    // Calculate error
    if (fabsf(result->float_result) > 1e-6f) {
        result->error_pct = 100.0f * fabsf(result->float_result - result->fixed_result) / fabsf(result->float_result);
    } else {
        result->error_pct = 0.0f;
    }
}

/**
 * @brief Test 3: IIR Filter
 */
static void test_iir_filter(TestResults_t *result) {
    uint32_t start, end;

    // Float IIR setup
    IIRFilter_t float_filter;
    iir_filter_init(&float_filter, 0.3f);

    // Fixed IIR setup
    FixedIIRFilter_t fixed_filter;
    fixed_iir_filter_init(&fixed_filter, FLOAT_TO_FIXED(0.3f));

    volatile float float_result = 0.0f;
    volatile fixed_t fixed_result = 0;

    // Floating-point IIR test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        float_result = iir_filter_update(&float_filter, float_input[i % TEST_ARRAY_SIZE]);
    }
    end = dwt_get_cycles();
    result->float_cycles = end - start;
    result->float_result = float_result;

    // Fixed-point IIR test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        fixed_result = fixed_iir_filter_update(&fixed_filter, fixed_input[i % TEST_ARRAY_SIZE]);
    }
    end = dwt_get_cycles();
    result->fixed_cycles = end - start;
    result->fixed_result = FIXED_TO_FLOAT(fixed_result);

    // Calculate error
    if (fabsf(result->float_result) > 1e-6f) {
        result->error_pct = 100.0f * fabsf(result->float_result - result->fixed_result) / fabsf(result->float_result);
    } else {
        result->error_pct = 0.0f;
    }
}

/**
 * @brief Test 4: Trigonometric Functions
 */
static void test_trig_functions(TestResults_t *result) {
    uint32_t start, end;
    volatile float float_result = 0.0f;
    volatile fixed_t fixed_result = 0;

    // Floating-point trig test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        float angle = float_input[i % TEST_ARRAY_SIZE];
        float_result = sinf(angle) + cosf(angle);
    }
    end = dwt_get_cycles();
    result->float_cycles = end - start;
    result->float_result = float_result;

    // Fixed-point trig test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        fixed_t angle = fixed_input[i % TEST_ARRAY_SIZE];
        fixed_result = fixed_add(fixed_sin(angle), fixed_cos(angle));
    }
    end = dwt_get_cycles();
    result->fixed_cycles = end - start;
    result->fixed_result = FIXED_TO_FLOAT(fixed_result);

    // Calculate error
    if (fabsf(result->float_result) > 1e-6f) {
        result->error_pct = 100.0f * fabsf(result->float_result - result->fixed_result) / fabsf(result->float_result);
    } else {
        result->error_pct = 0.0f;
    }
}

/**
 * @brief Test 5: Square Root
 */
static void test_sqrt(TestResults_t *result) {
    uint32_t start, end;
    volatile float float_result = 0.0f;
    volatile fixed_t fixed_result = 0;

    // Floating-point sqrt test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        float val = fabsf(float_input[i % TEST_ARRAY_SIZE]);
        float_result = sqrtf(val);
    }
    end = dwt_get_cycles();
    result->float_cycles = end - start;
    result->float_result = float_result;

    // Fixed-point sqrt test
    overflow_count = 0;
    start = dwt_get_cycles();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        fixed_t val = fixed_abs(fixed_input[i % TEST_ARRAY_SIZE]);
        fixed_result = fixed_sqrt(val);
    }
    end = dwt_get_cycles();
    result->fixed_cycles = end - start;
    result->fixed_result = FIXED_TO_FLOAT(fixed_result);

    // Calculate error
    if (fabsf(result->float_result) > 1e-6f) {
        result->error_pct = 100.0f * fabsf(result->float_result - result->fixed_result) / fabsf(result->float_result);
    } else {
        result->error_pct = 0.0f;
    }
}

/**
 * @brief Print test results
 */
static void print_test_results(const char *test_name, TestResults_t *result) {
    // Calculate speedup
    float speedup = (float)result->float_cycles / (float)result->fixed_cycles;

    printf("\r\n=== %s ===\r\n", test_name);
    printf("Float cycles:  %lu\r\n", result->float_cycles);
    printf("Fixed cycles:  %lu\r\n", result->fixed_cycles);
    printf("Speedup:       %.2fx\r\n", (double)speedup);
    printf("Float result:  %.6f\r\n", (double)result->float_result);
    printf("Fixed result:  %.6f\r\n", (double)result->fixed_result);
    printf("Error:         %.4f%%\r\n", (double)result->error_pct);
}

/**
 * @brief Main test function
 */
int main(void) {
    TestResults_t results;

    // Setup system clocks
    setup();

    // Initialize semihosting for printf
    initialise_monitor_handles();

    // Initialize cycle counter
    dwt_init();

    // Initialize test data
    init_test_data();

    printf("\r\n\r\n====================================\r\n");
    printf("STM32G071RB Float vs Fixed-Point Test\r\n");
    printf("Clock: 64 MHz, Cortex-M0+ (no FPU)\r\n");
    printf("====================================\r\n");

    // Run tests
    while (true) {
        printf("\r\n\r\nStarting test run...\r\n");

        // Test 1: Basic Arithmetic
        test_basic_arithmetic(&results);
        print_test_results("Basic Arithmetic", &results);

        // Test 2: PID Controller
        test_pid_controller(&results);
        print_test_results("PID Controller", &results);

        // Test 3: IIR Filter
        test_iir_filter(&results);
        print_test_results("IIR Filter", &results);

        // Test 4: Trigonometric Functions
        test_trig_functions(&results);
        print_test_results("Trig Functions", &results);

        // Test 5: Square Root
        test_sqrt(&results);
        print_test_results("Square Root", &results);

        printf("\r\nTest run complete. Waiting 2 seconds...\r\n");

        // Add delay between test runs
        for (volatile int i = 0; i < 10000000; i++) {}
    }

    return 0;
}
