#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// Declare your test functions here
void test_camera();
void test_physics();
void test_utils();

// Define a struct to map test names to functions
typedef struct {
    const char *name;
    void (*test_func)();
} Test;

// Register your tests
Test tests[] = {
    {"camera", test_camera},
    {"physics", test_physics},
    {"utils", test_utils},
};

// Helper to get the number of tests
#define NUM_TESTS (sizeof(tests) / sizeof(tests[0]))

// Function to run a specific test by name
bool run_test(const char *test_name) {
    for (size_t i = 0; i < NUM_TESTS; ++i) {
        if (strcmp(tests[i].name, test_name) == 0) {
            printf("Running %s test...\n", test_name);
            tests[i].test_func();
            printf("%s test passed!\n", test_name);
            return true;
        }
    }
    return false;
}

// Function to run all tests
void run_all_tests() {
    size_t failed_tests = 0;
    for (size_t i = 0; i < NUM_TESTS; ++i) {
        printf("Running %s test...\n", tests[i].name);
        tests[i].test_func();
        printf("%s test passed!\n", tests[i].name);
    }
    printf("All tests passed!\n");
}

int main(int argc, char **argv) {
    if (argc == 1) {
        // Run all tests if no arguments are passed
        printf("Running all tests...\n");
        run_all_tests();
        return 0;
    }

    // Run specific test
    if (!run_test(argv[1])) {
        fprintf(stderr, "Error: Unknown test '%s'\n", argv[1]);
        fprintf(stderr, "Available tests:\n");
        for (size_t i = 0; i < NUM_TESTS; ++i) {
            fprintf(stderr, "  %s\n", tests[i].name);
        }
        return 1; // Non-zero exit for error
    }

    return 0; // Zero exit for success
}

