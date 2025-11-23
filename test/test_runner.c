#include "test_utils.h"

// Forward declarations for test registration functions
void register_pid_tests(void);

int main(void) {
    printf("RobotCar Unit Test Suite\n");
    printf("=======================\n\n");
    
    // Register all test suites
    register_pid_tests();
    
    // Run all tests
    run_all_tests();
    
    // Return exit code based on test results
    return (test_failures > 0) ? 1 : 0;
}

