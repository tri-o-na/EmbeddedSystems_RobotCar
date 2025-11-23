#include "test_utils.h"
#include "pid.h"
#include <math.h>

// Test PID initialization
void test_pid_init(void) {
    PID pid;
    pid_init(&pid, 1.0f, 0.5f, 0.1f, -10.0f, 10.0f);
    
    TEST_ASSERT(fabsf(pid.kp - 1.0f) < 0.001f);
    TEST_ASSERT(fabsf(pid.ki - 0.5f) < 0.001f);
    TEST_ASSERT(fabsf(pid.kd - 0.1f) < 0.001f);
    TEST_ASSERT(fabsf(pid.integral - 0.0f) < 0.001f);
    TEST_ASSERT(fabsf(pid.prev_error - 0.0f) < 0.001f);
    TEST_ASSERT(fabsf(pid.out_min - (-10.0f)) < 0.001f);
    TEST_ASSERT(fabsf(pid.out_max - 10.0f) < 0.001f);
    
    TEST_PASS();
}

// Test PID with proportional term only
void test_pid_proportional_only(void) {
    PID pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.setpoint = 10.0f;
    
    // Current value is 5.0, error should be 5.0
    // Output should be kp * error = 2.0 * 5.0 = 10.0
    float output = pid_update(&pid, 5.0f);
    TEST_ASSERT_FLOAT_EQ(10.0f, output, 0.001f);
    
    // Current value is 15.0, error should be -5.0
    // Output should be kp * error = 2.0 * (-5.0) = -10.0
    output = pid_update(&pid, 15.0f);
    TEST_ASSERT_FLOAT_EQ(-10.0f, output, 0.001f);
    
    TEST_PASS();
}

// Test PID with integral term
void test_pid_integral_term(void) {
    PID pid;
    pid_init(&pid, 1.0f, 0.5f, 0.0f, -100.0f, 100.0f);
    pid.setpoint = 10.0f;
    
    // First update: error = 5.0, integral = 5.0
    // output = 1.0 * 5.0 + 0.5 * 5.0 = 5.0 + 2.5 = 7.5
    float output1 = pid_update(&pid, 5.0f);
    TEST_ASSERT_FLOAT_EQ(7.5f, output1, 0.001f);
    
    // Second update: error still 5.0, integral = 10.0
    // output = 1.0 * 5.0 + 0.5 * 10.0 = 5.0 + 5.0 = 10.0
    float output2 = pid_update(&pid, 5.0f);
    TEST_ASSERT_FLOAT_EQ(10.0f, output2, 0.001f);
    
    TEST_PASS();
}

// Test PID with derivative term
void test_pid_derivative_term(void) {
    PID pid;
    pid_init(&pid, 1.0f, 0.0f, 2.0f, -100.0f, 100.0f);
    pid.setpoint = 10.0f;
    
    // First update: error = 5.0, derivative = 5.0 (no prev_error)
    // output = 1.0 * 5.0 + 2.0 * 5.0 = 5.0 + 10.0 = 15.0
    float output1 = pid_update(&pid, 5.0f);
    TEST_ASSERT_FLOAT_EQ(15.0f, output1, 0.001f);
    
    // Second update: error = 3.0, derivative = 3.0 - 5.0 = -2.0
    // output = 1.0 * 3.0 + 2.0 * (-2.0) = 3.0 - 4.0 = -1.0
    float output2 = pid_update(&pid, 7.0f);
    TEST_ASSERT_FLOAT_EQ(-1.0f, output2, 0.001f);
    
    TEST_PASS();
}

// Test PID output clamping
void test_pid_output_clamping(void) {
    PID pid;
    pid_init(&pid, 10.0f, 0.0f, 0.0f, -5.0f, 5.0f);
    pid.setpoint = 10.0f;
    
    // Large error should be clamped
    // error = 10.0, output = 10.0 * 10.0 = 100.0, should clamp to 5.0
    float output = pid_update(&pid, 0.0f);
    TEST_ASSERT_FLOAT_EQ(5.0f, output, 0.001f);
    
    // Negative large error should clamp to minimum
    // error = -10.0, output = 10.0 * (-10.0) = -100.0, should clamp to -5.0
    pid.setpoint = 0.0f;
    output = pid_update(&pid, 10.0f);
    TEST_ASSERT_FLOAT_EQ(-5.0f, output, 0.001f);
    
    TEST_PASS();
}

// Test PID convergence to setpoint
void test_pid_convergence(void) {
    PID pid;
    pid_init(&pid, 0.5f, 0.1f, 0.05f, -10.0f, 10.0f);
    pid.setpoint = 10.0f;
    
    float current = 0.0f;
    float output;
    
    // Simulate multiple updates to see convergence
    for (int i = 0; i < 10; i++) {
        output = pid_update(&pid, current);
        // Apply output to current (simplified model)
        current += output * 0.1f;
    }
    
    // After multiple iterations, current should be closer to setpoint
    // This is a simplified test - in reality, convergence depends on the system
    TEST_ASSERT(fabsf(current - pid.setpoint) < fabsf(0.0f - pid.setpoint));
    
    TEST_PASS();
}

// Test PID with zero setpoint
void test_pid_zero_setpoint(void) {
    PID pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    pid.setpoint = 0.0f;
    
    // Current is 5.0, error = 0.0 - 5.0 = -5.0
    float output = pid_update(&pid, 5.0f);
    TEST_ASSERT_FLOAT_EQ(-5.0f, output, 0.001f);
    
    TEST_PASS();
}

// Test PID integral windup prevention (indirectly through clamping)
void test_pid_integral_windup(void) {
    PID pid;
    pid_init(&pid, 1.0f, 10.0f, 0.0f, -5.0f, 5.0f);
    pid.setpoint = 10.0f;
    
    // Large integral term should be clamped
    // Multiple updates with same error
    for (int i = 0; i < 5; i++) {
        pid_update(&pid, 0.0f);
    }
    
    // Output should be clamped, not growing unbounded
    float output = pid_update(&pid, 0.0f);
    TEST_ASSERT(output <= 5.0f);
    TEST_ASSERT(output >= -5.0f);
    
    TEST_PASS();
}

// Register all PID tests
void register_pid_tests(void) {
    register_test("PID Init", test_pid_init);
    register_test("PID Proportional Only", test_pid_proportional_only);
    register_test("PID Integral Term", test_pid_integral_term);
    register_test("PID Derivative Term", test_pid_derivative_term);
    register_test("PID Output Clamping", test_pid_output_clamping);
    register_test("PID Convergence", test_pid_convergence);
    register_test("PID Zero Setpoint", test_pid_zero_setpoint);
    register_test("PID Integral Windup", test_pid_integral_windup);
}

