#!/bin/sh

BUILD_DIR="build"
EXECUTABLE="./build/app/my_executable"

if [ -n "$1" ] && [ $1 = "clean" ]; then
    echo "Cleaning build directory."
    rm -rf $BUILD_DIR
fi


if [ ! -d $BUILD_DIR ]; then
    echo "Directory build does not exist."
    mkdir build
fi

cd $BUILD_DIR

cmake -G "Unix Makefiles" ..

cmake --build .

cd ..

# if [ -f $EXECUTABLE ]; then
#     echo "Executable found."
#     $EXECUTABLE
# else
#     echo "Executable not found."
# fi