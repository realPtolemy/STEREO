cmake_minimum_required(VERSION 3.10)
project(app)

file(GLOB SOURCES "src/mapper/*.cpp" "src/tracker/*.cpp" "main.cpp")

add_executable(my_executable ${SOURCES})

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INSTALL_DIR}/include/eigen3
)
target_link_libraries(my_executable PRIVATE ${OpenCV_LIBS} cartesian3dgrid)