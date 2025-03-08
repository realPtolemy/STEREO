#!/bin/sh

BUILD_DIR="build"
EXECUTABLE="./app/my_executable"

if [ $1 = "clean" ]; then
    echo "Cleaning build directory."
    rm -rf $BUILD_DIR
fi


if [ ! -d $BUILD_DIR ]; then
    echo "Directory build does not exist."
    mkdir build
fi

cd $BUILD_DIR

cmake ..

cmake --build .

if [ -f $EXECUTABLE ]; then
    echo "Executable found."
    $EXECUTABLE
else
    echo "Executable not found."
fi
