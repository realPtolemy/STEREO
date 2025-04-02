cmake_minimum_required(VERSION 3.10)
project(app)


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
    talking_test/src/class1.cpp
    talking_test/src/class2.cpp
    talking_test/src/coordinator.cpp
    talking_test/src/common.cpp
)

add_executable(my_executable ${SOURCES})

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mapper/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracker/include
    ${CMAKE_CURRENT_SOURCE_DIR}/talking_test/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INSTALL_DIR}/include/eigen3
    ${SOPHUS_INSTALL_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/minkindr/include
)
target_link_libraries(my_executable PRIVATE ${OpenCV_LIBS} cartesian3dgrid ${PCL_LIBRARIES} tf2)