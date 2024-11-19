# Enable testing
enable_testing()

# Define a macro to reduce repetitive code for adding test executables
macro(add_test_executable test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} PRIVATE physics tree utils raylib)
    target_include_directories(${test_name} PRIVATE ${CMAKE_SOURCE_DIR}/src)
    add_test(NAME ${test_name} COMMAND ${test_name})
endmacro()

# Add test executables
add_test_executable(test_orb_elems_from_rv test_orb_elems_from_rv.c)
add_test_executable(test_rv_from_orb_elems_circular_equatorial test_rv_from_orb_elems_circular_equatorial.c)
add_test_executable(test_rv_from_orb_elems_circular_inclined test_rv_from_orb_elems_circular_inclined.c)
add_test_executable(test_rv_from_orb_elems_failure test_rv_from_orb_elems_failure.c)
add_test_executable(test_intermediate_consistency test_intermediate_consistency.c)
add_test_executable(test_perifocal_to_inertial_and_back test_perifocal_to_inertial_and_back.c)
add_test_executable(test_orb_elems_from_rv_against_orb_mech_eng_students test_orb_elems_from_rv_against_orb_mech_eng_students.c)
add_test_executable(test_rv_from_orb_elems_orb_mech_eng_students test_rv_from_orb_elems_orb_mech_eng_students.c)
