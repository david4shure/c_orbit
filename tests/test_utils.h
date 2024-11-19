#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdio.h>
#include <math.h>

// Epsilon for floating-point comparison
#define EPSILON 1e-6

// Helper macros for assertions
#define ASSERT_EQ_DOUBLE(expected, actual) \
    do { \
        if (fabs((expected) - (actual)) > EPSILON) { \
            printf("[FAIL] %s:%d: Expected %f, got %f\n", __FILE__, __LINE__, (expected), (actual)); \
        } else { \
            printf("[PASS] %s:%d: %f == %f\n", __FILE__, __LINE__, (expected), (actual)); \
        } \
    } while (0)

#define ASSERT_EQ_VEC3(expected, actual) \
    do { \
        ASSERT_EQ_DOUBLE((expected).x, (actual).x); \
        ASSERT_EQ_DOUBLE((expected).y, (actual).y); \
        ASSERT_EQ_DOUBLE((expected).z, (actual).z); \
    } while (0)

#endif // TEST_UTILS_H

