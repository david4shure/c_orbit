cmake_minimum_required(VERSION 3.12...3.30)
project(c_orbit C)

include(FetchContent)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
set(RAYLIB_VERSION 5.0)

FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
)

FetchContent_MakeAvailable(raylib)

set(RAYGUI_VERSION 4.0)

FetchContent_Declare(
    raygui
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raygui/archive/refs/tags/${RAYGUI_VERSION}.tar.gz
)

FetchContent_MakeAvailable(raygui)

enable_testing()

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)

