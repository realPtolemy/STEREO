cmake_minimum_required(VERSION 3.10)
project(unitree_legged_sdk)

add_library(unitree_legged_sdk INTERFACE)

target_include_directories(minkindr INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)