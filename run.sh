#!/bin/sh

BUILD_DIR="build"
EXECUTABLE="./build/app/my_executable"

ACTION1="$1"
ACTION2="$2"

# Default to 'run' if no argument is given
if [ -z "$ACTION1" ]; then
    ACTION1="run"
fi

# Clean
if [ "$ACTION1" = "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Build (includes mkdir + cmake config)
if [ "$ACTION1" = "clean" ] || [ "$ACTION1" = "build" ]; then
    if [ ! -d "$BUILD_DIR" ]; then
        echo "Creating build directory..."
        mkdir "$BUILD_DIR"
    fi
    echo "Running CMake configuration..."
    cd "$BUILD_DIR"
    cmake -G Ninja ..
    cd ..
fi

# Link (compile)
if [ "$ACTION1" = "clean" ] || [ "$ACTION1" = "build" ] || [ "$ACTION1" = "compile" ]; then
    echo "Compiling project..."
    cd "$BUILD_DIR"
    cmake --build .
    cd ..
fi

if [ "$ACTION1" = "test" ] || [ "$ACTION2" = "test" ]; then
    echo "Running tests..."
    cd "$BUILD_DIR"
    ctest --output-on-failure
    cd ..
    exit 0
fi

# Run
if [ -f "$EXECUTABLE" ]; then
    echo "Running executable..."
    "$EXECUTABLE"
else
    echo "Executable not found."
fi