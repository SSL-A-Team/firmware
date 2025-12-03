# Float-Test

A comprehensive test program for evaluating floating-point vs fixed-point computation performance on the STM32G071RB microcontroller (Nucleo-G071RB board).

## Overview

This project provides a comparative analysis of floating-point and fixed-point arithmetic on the STM32G071RB Cortex-M0+ microcontroller, which lacks a hardware floating-point unit (FPU). The tests help determine whether fixed-point arithmetic offers significant performance benefits for embedded motor control applications.

## Features

### Fixed-Point Implementation
- **Q16.16 format**: 16-bit integer part, 16-bit fractional part
- Full arithmetic operations: add, subtract, multiply, divide
- Advanced functions: square root, sine, cosine
- PID controller implementation
- IIR filter implementation

### Performance Tests

1. **Basic Arithmetic Operations**
   - Multiplication, division, addition, subtraction
   - Mixed operations

2. **PID Controller**
   - Proportional-Integral-Derivative control
   - Real-world motor control simulation
   - Measures loop execution time

3. **IIR Filter**
   - First-order low-pass filter
   - Alpha parameter: 0.3
   - Common signal processing operation

4. **Trigonometric Functions**
   - Sine and cosine calculations
   - Taylor series approximation for fixed-point

5. **Square Root**
   - Newton's method implementation
   - Common in magnitude calculations

## Building

### From project root:
```bash
make float-test--float-comparison
```

### Or from float-test directory:
```bash
cd float-test
cmake -B build
cd build
make float-comparison
```

## Running Tests End-to-End

### Complete Test Workflow

1. **Build the test program**
   ```bash
   cd float-test
   cmake -B build
   cd build
   make float-comparison
   ```

2. **Connect Hardware**
   - Connect the Nucleo-G071RB board via USB
   - The ST-Link debugger will be automatically detected

3. **Start OpenOCD (in background)**
   ```bash
   cd build
   openocd -f ../../util/openocd/nucleo-g071rb.cfg &> openocd_output.log &
   ```

4. **Program and Run Tests**
   ```bash
   arm-none-eabi-gdb -batch -x run_test.gdb bin/float-comparison
   ```

   Or for interactive debugging:
   ```bash
   arm-none-eabi-gdb bin/float-comparison
   (gdb) target extended-remote :3333
   (gdb) monitor arm semihosting enable
   (gdb) load
   (gdb) monitor reset halt
   (gdb) continue
   ```

5. **View Test Results**

   Test output is streamed via ARM semihosting to the OpenOCD log:
   ```bash
   tail -f openocd_output.log
   ```

   You'll see cycle counts, speedup ratios, and accuracy metrics for each test:
   ```
   === Basic Arithmetic ===
   Float cycles:  16533822
   Fixed cycles:  20062563
   Speedup:       0.82x
   Float result:  12.499596
   Fixed result:  12.499512
   Error:         0.0007%
   ```

6. **Stop OpenOCD**
   ```bash
   killall openocd
   ```

### Quick Test Script

The GDB script `run_test.gdb` in the build directory automates programming and execution:
```gdb
target extended-remote :3333
monitor arm semihosting enable
load
monitor reset halt
continue
quit
```

### Alternative: From Project Root

Program the target device:
```bash
make float-test--float-comparison--prog
```

Attach GDB debugger:
```bash
make float-test--float-comparison--debug
```

## Measurement Methodology

The test program uses the SysTick timer with overflow tracking to measure execution time in CPU cycles. Each test:

1. Resets the overflow counter to zero
2. Initializes test data arrays
3. Runs floating-point implementation for N iterations
4. Records CPU cycles used (with overflow tracking)
5. Resets overflow counter
6. Runs fixed-point implementation for N iterations
7. Records CPU cycles used (with overflow tracking)
8. Calculates speedup ratio and accuracy error
9. Outputs results via ARM semihosting (printf to OpenOCD)

The SysTick overflow tracking allows accurate measurement of tests taking longer than 262ms (16.7M cycles @ 64MHz), solving the 24-bit timer limitation on Cortex-M0+.

## Measured Results

On Cortex-M0+ @ 64MHz without hardware FPU (100 iterations each):

### Fixed-Point Wins:
- **IIR Filter**: 1.84x faster (128K vs 70K cycles)
- **Trigonometric Functions**: 1.42x faster (1.4M vs 961K cycles)
- **PID Controller**: 1.10x faster (349K vs 318K cycles)

### Floating-Point Wins:
- **Square Root**: 4.2x faster (141K vs 588K cycles) - GCC's sqrtf is highly optimized
- **Basic Arithmetic**: 1.21x faster (16.5M vs 20M cycles)

### Accuracy:
- **Q16.16 fixed-point** provides excellent accuracy for all tests
- Error rates: 0.0007% to 4.7% depending on operation
- Most operations: <0.01% error

### Key Insights:
- Fixed-point excels at filters, trig, and control loops
- Software floating-point is competitive for complex arithmetic
- GCC's math library is well-optimized for Cortex-M0+
- Fixed-point requires careful overflow management but offers predictable performance

## Target Hardware

- **Board**: Nucleo-G071RB
- **Microcontroller**: STM32G071RB
- **Core**: ARM Cortex-M0+
- **Clock**: 64 MHz (HSI + PLL)
- **FPU**: None (software floating-point emulation)
- **Flash**: 128 KB
- **RAM**: 36 KB

## Project Structure

```
float-test/
├── bin/
│   └── float-comparison/      # Main test program
│       ├── main.c             # Test implementations
│       ├── main.h
│       ├── setup.c            # Clock configuration
│       ├── setup.h
│       └── time.h
├── common/                    # Shared utilities
│   ├── fixed_point.c          # Fixed-point math library
│   ├── fixed_point.h
│   ├── pid.c                  # Float PID implementation
│   ├── iir.c                  # Float IIR implementation
│   ├── startup_stm32g071xx.s  # Startup code for STM32G071
│   ├── stm32g071xx.ld         # Linker script for STM32G071
│   ├── stm32g071xx.h          # MCU definitions
│   ├── system_stm32g0xx.h     # System header
│   └── cmsis/                 # ARM CMSIS headers
├── CMakeLists.txt
└── README.md
```

## Integration with Motor Controller

This test program is designed to mirror the computational patterns used in the motor-controller project. Results can inform decisions about whether to use fixed-point arithmetic in performance-critical motor control loops.

## Configuration and Customization

### Test Parameters
Modify in `bin/float-comparison/main.c`:
- `NUM_ITERATIONS`: Number of iterations per test (default: 100)
- `TEST_ARRAY_SIZE`: Size of input data arrays (default: 100)
- Increase iterations for longer tests with more stable averages
- Decrease iterations for faster testing cycles

### Compiler Optimization
The CMake configuration uses `-O2` optimization. To test different levels:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug    # -O0
cmake -B build -DCMAKE_BUILD_TYPE=Release  # -O2
```

### Output Options
- **Semihosting (default)**: Printf output via OpenOCD - no extra hardware needed
- **UART**: Can be added for standalone operation without debugger
- **Breakpoint Inspection**: Set breakpoints after each test in GDB

### OpenOCD Configuration
The config file `util/openocd/nucleo-g071rb.cfg` includes:
- ST-Link interface setup
- Adapter speed: 1800 kHz (max for STM32G0)
- ARM semihosting enabled automatically
- Reset configuration for reliable programming

## Troubleshooting

### OpenOCD Connection Issues
```bash
# Check if OpenOCD is already running
ps aux | grep openocd

# Kill existing instances
killall openocd

# Check USB connection
lsusb | grep STM
```

### No Semihosting Output
- Ensure OpenOCD config enables semihosting: `arm semihosting enable`
- Check that binary is built with `--specs=rdimon.specs -lrdimon`
- Verify `initialise_monitor_handles()` is called in main()
- Look for output in the OpenOCD log file, not in GDB

### Timer Overflow Issues
- The implementation tracks SysTick overflows automatically via interrupt
- For very long tests (>10 seconds), consider reducing iterations
- Check that `SysTick_Handler()` is properly defined and not conflicting

### Build Errors
```bash
# Missing CMSIS headers
cd float-test/common/cmsis
# Ensure core_cm0plus.h and related files exist

# Linker errors - check linker script path
grep "stm32g071xx.ld" CMakeLists.txt
```

### Programming Failures
```bash
# Reset board by pressing the black reset button
# Try reducing adapter speed in OpenOCD config to 1000 kHz
# Check board power LED is on
# Verify USB cable supports data transfer (not charge-only)
```

### Test Results Look Wrong
- Ensure optimization is enabled (`-O2` in CMakeLists.txt)
- Check that overflow counter is reset between tests
- Verify NUM_ITERATIONS is appropriate (100 recommended)
- Very short tests may have measurement noise - increase iterations

## License

Same as parent firmware repository.
