#!/bin/sh

BUILD_DIR="build"
EXECUTABLE="./build/app/my_executable"

ACTION1="$1"
ACTION2="$2"

if [ -z "$ACTION1" ]; then
    ACTION1="run"
fi

if [ "$ACTION1" = "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

if [ "$ACTION1" = "clean" ] || [ "$ACTION1" = "build" ]; then
    if [ ! -d "$BUILD_DIR" ]; then
        echo "Creating build directory..."
        mkdir "$BUILD_DIR"
    fi
    echo "Running CMake configuration..."

    cd "$BUILD_DIR"
    if [ "$ACTION2" = "debug" ]; then
        cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..
    else
        cmake -G Ninja ..
    fi
    cd ..
fi

# Link (compile)
if [ "$ACTION1" = "clean" ] || [ "$ACTION1" = "build" ] || [ "$ACTION1" = "compile" ]; then
    echo "Compiling project..."
    cd "$BUILD_DIR"
    cmake --build . -- -j2
    cd ..
fi

# Run
if [ -f "$EXECUTABLE" ]; then
    echo "Running executable..."
    if [ "$ACTION2" = "debug" ]; then
        gdb "$EXECUTABLE"
    else
        "$EXECUTABLE"
    fi
else
    echo "Executable not found."
fi