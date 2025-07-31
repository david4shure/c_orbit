#!/bin/bash

echo "Building C Orbit Web Version..."

# Create build directory
mkdir -p docs
cd docs

# Download and setup raylib webassembly if not already present
if [ ! -d "raylib-5.5_webassembly" ]; then
    echo "Downloading raylib webassembly..."
    wget -O raylib-5.5_webassembly.zip https://github.com/raysan5/raylib/releases/download/5.5/raylib-5.5_webassembly.zip
    unzip raylib-5.5_webassembly.zip
    rm raylib-5.5_webassembly.zip
fi

# Copy missing headers
echo "Copying missing headers..."
cp ../_deps/raylib-src/src/rcamera.h raylib-5.5_webassembly/include/
cp ../_deps/raylib-src/src/rgestures.h raylib-5.5_webassembly/include/

# Copy resources
echo "Copying resources..."
mkdir -p resources
cp -r ../src/resources/* resources/

# Compile game
echo "Compiling game..."
emcc \
    ../src/main_web.c \
    ../src/physics/corbit_math.c \
    ../src/physics/kepler.c \
    ../src/physics/orbital_lines.c \
    ../src/physics/propagation.c \
    ../src/physics/time.c \
    ../src/tree/tree.c \
    ../src/utils/darray.c \
    ../src/utils/logger.c \
    ../src/utils/rlutil.c \
    ../src/camera.c \
    raylib-5.5_webassembly/lib/libraylib.a \
    -o index.html \
    --shell-file ../web_shell.html \
    -I../src -I../src/physics -I../src/tree -I../src/utils \
    -Iraylib-5.5_webassembly/include \
    -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 \
    -s USE_WEBGL2=0 -s FULL_ES2=1 \
    -s USE_GLFW=3 -s ALLOW_MEMORY_GROWTH=1 \
    -s EXPORTED_RUNTIME_METHODS='["ccall","cwrap"]' \
    -s EXPORTED_FUNCTIONS='["_main"]' \
                    -s NO_EXIT_RUNTIME=1 -s ASSERTIONS=1 -s SAFE_HEAP=1 \
                -s TOTAL_MEMORY=67108864 -s INITIAL_MEMORY=33554432 \
                --preload-file resources \
                -O2

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "Open docs/index.html in a web browser."
    echo "Note: You'll need to serve the files from a web server due to CORS restrictions."
    echo "You can use: python3 -m http.server 8000"
else
    echo "❌ Build failed"
    exit 1
fi
