#!/bin/sh

BUILD_DIR="build"
EXECUTABLE="./build/app/my_executable"

ACTION="$1"

# Default to 'run' if no argument is given
if [ -z "$ACTION" ]; then
    ACTION="run"
fi

# Clean
if [ "$ACTION" = "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Build (includes mkdir + cmake config)
if [ "$ACTION" = "clean" ] || [ "$ACTION" = "build" ]; then
    if [ ! -d "$BUILD_DIR" ]; then
        echo "Creating build directory..."
        mkdir "$BUILD_DIR"
    fi
    echo "Running CMake configuration..."
    cd "$BUILD_DIR"
    cmake -G "Unix Makefiles" ..
    cd ..
fi

# Link (compile)
if [ "$ACTION" = "clean" ] || [ "$ACTION" = "build" ] || [ "$ACTION" = "compile" ]; then
    echo "Compiling project..."
    cd "$BUILD_DIR"
    cmake --build . -j$(nproc)
    cd ..
fi

# Run
if [ -f "$EXECUTABLE" ]; then
    echo "Running executable..."
    "$EXECUTABLE"
else
    echo "Executable not found."
fi