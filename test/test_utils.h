#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

// Simple test framework macros
#define TEST_ASSERT(condition) \
    do { \
        if (!(condition)) { \
            printf("FAIL: %s:%d: Assertion failed: %s\n", __FILE__, __LINE__, #condition); \
            test_failures++; \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQ(expected, actual, tolerance) \
    do { \
        float diff = fabsf((expected) - (actual)); \
        if (diff > (tolerance)) { \
            printf("FAIL: %s:%d: Expected %.6f, got %.6f (diff: %.6f)\n", \
                   __FILE__, __LINE__, (expected), (actual), diff); \
            test_failures++; \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_INT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            printf("FAIL: %s:%d: Expected %d, got %d\n", \
                   __FILE__, __LINE__, (expected), (actual)); \
            test_failures++; \
            return; \
        } \
    } while(0)

#define TEST_ASSERT_UINT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            printf("FAIL: %s:%d: Expected %u, got %u\n", \
                   __FILE__, __LINE__, (unsigned)(expected), (unsigned)(actual)); \
            test_failures++; \
            return; \
        } \
    } while(0)

#define TEST_PASS() \
    do { \
        printf("PASS: %s\n", __func__); \
        test_passes++; \
    } while(0)

// Test statistics
extern int test_passes;
extern int test_failures;

// Test function pointer type
typedef void (*test_func_t)(void);

// Test registration
void register_test(const char *name, test_func_t func);
void run_all_tests(void);

#endif // TEST_UTILS_H

