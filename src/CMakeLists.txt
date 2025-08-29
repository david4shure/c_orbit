# Add each component as a subdirectory
add_subdirectory(physics)
add_subdirectory(tree)
add_subdirectory(utils)

# Define the main executable and link the libraries
add_executable(c_orbit main.c camera.c)

# Link libraries
target_link_libraries(c_orbit PRIVATE tree physics utils raylib)

# Include directories for main executable
target_include_directories(c_orbit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(c_orbit PRIVATE ${raylib_SOURCE_DIR}/src)
target_include_directories(c_orbit PRIVATE ${raygui_SOURCE_DIR}/src)

# Copy resources
if ("${PLATFORM}" STREQUAL "Web")
    add_custom_command(
        TARGET c_orbit PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/resources $<TARGET_FILE_DIR:c_orbit>/../resources
    )
else()
    add_custom_command(
        TARGET c_orbit POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/resources $<TARGET_FILE_DIR:c_orbit>/resources
    )
endif()

