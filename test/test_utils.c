#include "test_utils.h"
#include <string.h>

#define MAX_TESTS 100

typedef struct {
    const char *name;
    test_func_t func;
} test_entry_t;

static test_entry_t tests[MAX_TESTS];
static int test_count = 0;

int test_passes = 0;
int test_failures = 0;

void register_test(const char *name, test_func_t func) {
    if (test_count < MAX_TESTS) {
        tests[test_count].name = name;
        tests[test_count].func = func;
        test_count++;
    }
}

void run_all_tests(void) {
    printf("\n=== Running RobotCar Unit Tests ===\n\n");
    
    for (int i = 0; i < test_count; i++) {
        printf("[%d/%d] Running: %s\n", i + 1, test_count, tests[i].name);
        test_failures = 0;  // Reset for each test
        test_passes = 0;
        
        tests[i].func();
        
        if (test_failures == 0) {
            printf("  ✓ PASSED\n");
        } else {
            printf("  ✗ FAILED (%d assertion(s) failed)\n", test_failures);
        }
        printf("\n");
    }
    
    printf("=== Test Summary ===\n");
    printf("Total tests: %d\n", test_count);
    printf("Passed: %d\n", test_passes);
    printf("Failed: %d\n", test_failures);
    
    if (test_failures == 0) {
        printf("\n✓ All tests passed!\n");
    } else {
        printf("\n✗ Some tests failed!\n");
    }
}

