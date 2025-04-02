cmake_minimum_required(VERSION 3.20)
project(tf2)

set(
    SOURCES
    src/buffer_core.cpp
    src/cache.cpp
    src/static_cache.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

# Add the include directories
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against the common library for Time, etc.
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
)