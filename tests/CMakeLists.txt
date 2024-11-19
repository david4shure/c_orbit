# Enable testing
enable_testing()

# List of test sources
set(TEST_SOURCES
    test_harness.c
    test_camera.c
    test_physics.c
    test_utils.c
)

# Define the test harness executable
add_executable(test_harness ${TEST_SOURCES})

# Link test harness to the libraries
target_link_libraries(test_harness PRIVATE physics tree utils raylib)

# Include directories for headers
target_include_directories(test_harness PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_test(NAME CameraTests COMMAND test_harness camera)
add_test(NAME PhysicsTests COMMAND test_harness physics)
add_test(NAME UtilsTests COMMAND test_harness utils)

