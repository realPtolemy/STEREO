cmake_minimum_required(VERSION 3.20)
project(MyProject)

include(FetchContent)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/3party")

include(Dependencies)

add_subdirectory(external)
add_subdirectory(app)