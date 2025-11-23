# RobotCar Unit Tests

This directory contains unit tests for the RobotCar embedded system project.

## Test Framework

The tests use a simple, lightweight test framework (`test_utils.h`) that doesn't require external dependencies. This makes it suitable for embedded systems testing.

## Building and Running Tests

### Using CMake (Recommended)

```bash
# From the project root directory
mkdir -p test_build
cd test_build
cmake ../test
make
./robotcar_tests
```

### Manual Compilation

```bash
gcc -std=c11 -Wall -Wextra -g -O0 \
    test_runner.c \
    test_pid.c \
    test_utils.c \
    ../src/pid.c \
    -o robotcar_tests \
    -lm
./robotcar_tests
```

## Test Structure

- `test_utils.h/c` - Simple test framework with assertion macros
- `test_runner.c` - Main test runner that executes all tests
- `test_pid.c` - PID controller unit tests
- `CMakeLists.txt` - CMake configuration for building tests

## Adding New Tests

1. Create a new test file (e.g., `test_motor.c`)
2. Include `test_utils.h`
3. Write test functions using the assertion macros:
   - `TEST_ASSERT(condition)` - General assertion
   - `TEST_ASSERT_FLOAT_EQ(expected, actual, tolerance)` - Float comparison
   - `TEST_ASSERT_INT_EQ(expected, actual)` - Integer comparison
   - `TEST_PASS()` - Mark test as passed
4. Create a registration function:
   ```c
   void register_motor_tests(void) {
       register_test("Motor Init", test_motor_init);
       register_test("Motor Set", test_motor_set);
   }
   ```
5. Call the registration function in `test_runner.c`

## Test Macros

- `TEST_ASSERT(condition)` - Fails if condition is false
- `TEST_ASSERT_FLOAT_EQ(expected, actual, tolerance)` - Compares floats with tolerance
- `TEST_ASSERT_INT_EQ(expected, actual)` - Compares integers
- `TEST_ASSERT_UINT_EQ(expected, actual)` - Compares unsigned integers
- `TEST_PASS()` - Marks test as passed (call at end of test)

## Example Test

```c
void test_example(void) {
    int value = 5;
    TEST_ASSERT(value == 5);
    TEST_ASSERT_INT_EQ(5, value);
    TEST_PASS();
}
```

## Current Test Coverage

- ✅ PID Controller
  - Initialization
  - Proportional term
  - Integral term
  - Derivative term
  - Output clamping
  - Convergence behavior
  - Zero setpoint handling
  - Integral windup prevention

## Future Tests to Add

- Motor encoder functions (requires hardware mocking)
- Line following sensor logic
- Ultrasonic distance calculations
- IMU data processing
- Barcode decoding logic

