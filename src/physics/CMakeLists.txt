# Define the physics library
add_library(physics
    constants.h
    corbit_math.c
    corbit_math.h
    kepler.c
    kepler.h
    orbital_lines.c
    orbital_lines.h
    time.c
    time.h
)

# Specify include directories for the physics library
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(physics PUBLIC ${raylib_SOURCE_DIR}/src)
