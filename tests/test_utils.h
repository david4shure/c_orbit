#ifndef ASSERTIONS_H
#define ASSERTIONS_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../src/utils/logger.h"

// Equality assertion for integers
#define ASSERT_EQ(expected, actual) \
    InitializeLogger(DEBUG, true); \
    if ((expected) != (actual)) { \
        Error("[FAIL] %s:%d: Expected %d, got %d\n", __FILE__, __LINE__, (expected), (actual)); \
        exit(1); \
    } else { \
        Log("[PASS] %s:%d: %d == %d\n", __FILE__, __LINE__, (expected), (actual)); \
    }

// Near-equality assertion for floating-point numbers
#define ASSERT_NEAR(expected, actual, epsilon) \
    InitializeLogger(DEBUG, true); \
    if (fabs((expected) - (actual)) > (epsilon)) { \
        Error("[FAIL] %s:%d: Expected %f, got %f\n", __FILE__, __LINE__, (expected), (actual)); \
        exit(1); \
    } else { \
        Log("[PASS] %s:%d: %f == %f\n", __FILE__, __LINE__, (expected), (actual)); \
    }

// Vector equality assertion
#define ASSERT_EQ_DVECTOR3(expected, actual, epsilon) \
    do { \
        InitializeLogger(DEBUG, true); \
        Error("Expected Vector: (%.6f, %.6f, %.6f)\n", (expected).x, (expected).y, (expected).z); \
        Error("Actual Vector:   (%.6f, %.6f, %.6f)\n", (actual).x, (actual).y, (actual).z); \
        double x_diff = fabs((expected).x - (actual).x); \
        double y_diff = fabs((expected).y - (actual).y); \
        double z_diff = fabs((expected).z - (actual).z); \
        if (x_diff > epsilon || y_diff > epsilon || z_diff > epsilon) { \
            Error("[FAIL] %s:%d: Vector mismatch: |dx|=%.6f, |dy|=%.6f, |dz|=%.6f\n", \
                __FILE__, __LINE__, x_diff, y_diff, z_diff); \
            exit(1); \
        } else { \
            Info("[PASS] %s:%d: Vector match within epsilon %.6f\n", __FILE__, __LINE__, epsilon); \
        } \
    } while (0)

#endif // ASSERTIONS_H

