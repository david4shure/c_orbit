# Enable testing
enable_testing()

# Create executable for test_physics
add_executable(test_harness test_harness.c test_utils.h test_kepler.h test_kepler.c)

# Link to necessary libraries
target_link_libraries(test_harness PRIVATE physics tree utils raylib)

# Include directories for headers
target_include_directories(test_harness PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Add test to CTest
add_test(NAME PhysicsTests COMMAND test_harness)

