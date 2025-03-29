cmake_minimum_required(VERSION 3.10)
project(app)

# file(GLOB SOURCES "src/mapper/*.cpp" "src/tracker/*.cpp" "main.cpp")

set(SOURCES
    main.cpp
    mapper/src/mapper.cpp
    mapper/src/median_filtering.cpp
    mapper/src/process1_live.cpp
    mapper/src/utils.cpp
    mapper/src/mapper_emvs_stereo.cpp
    mapper/src/camera.cpp
    tracker/src/tracker.cpp
    tracker/src/lk_se3.cpp
)

add_executable(my_executable ${SOURCES})

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mapper/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracker/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INSTALL_DIR}/include/eigen3
    ${KINDR_INSTALL_DIR}/include
    ${SOPHUS_INSTALL_DIR}/include
)
target_link_libraries(my_executable PRIVATE ${OpenCV_LIBS} cartesian3dgrid ${PCL_LIBRARIES} tf2)