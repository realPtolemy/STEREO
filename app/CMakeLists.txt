cmake_minimum_required(VERSION 3.10)
project(app)

# Find dependencies
find_package(OpenCV REQUIRED)
set(PCL_DIR "${CMAKE_SOURCE_DIR}/3party/pcl_src/share/pcl-1.15")
#find_package(PCL 1.15 REQUIRED COMPONENTS common filters)
find_package(PCL 1.15 REQUIRED COMPONENTS common filters io)
set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/3party/boost_src")
#find_package(Boost REQUIRED COMPONENTS mpl system filesystem thread)
set(EIGEN3_ROOT "${CMAKE_SOURCE_DIR}/3party/eigen_src")
find_package(Eigen3 REQUIRED)
#find_package(Sophus REQUIRED)

# Fix Eigen alignment for PCL
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)

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
    udp/src/udp_start.cpp
    udp/src/udp.cpp
)

add_executable(my_executable ${SOURCES})

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/udp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mapper/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracker/include
    ${CMAKE_CURRENT_SOURCE_DIR}/talking_test/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/external/minkindr/include
    # ${CMAKE_SOURCE_DIR}/3party/pcl_src/include/pcl-1.15  # Temporary until PCL is fully configured
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(my_executable PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    cartesian3dgrid
    tf2
)

# Debug: Print variables to verify
message("PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message("PCL_LIBRARIES: ${PCL_LIBRARIES}")
message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")