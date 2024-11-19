# Define the physics library
add_library(utils
    darray.h
    darray.c
    logger.h
    logger.c
    rlutil.h
    rlutil.c
)

# Specify include directories for the physics library
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Include raylib headers
target_include_directories(utils PRIVATE ${raylib_SOURCE_DIR}/src)
