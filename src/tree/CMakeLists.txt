# Define the physics library
add_library(tree
    tree.h
    tree.c
)

# Specify include directories for the physics library
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Include raylib headers for colors and such
target_include_directories(tree PRIVATE ${raylib_SOURCE_DIR}/src)
