cmake_minimum_required(VERSION 3.10)
project(minkindr)

# Header-only library
add_library(minkindr INTERFACE)

# This makes #include "transform-2d.h" etc. work
target_include_directories(minkindr INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)