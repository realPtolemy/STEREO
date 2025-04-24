cmake_minimum_required(VERSION 3.10)
project(app)

option(ENABLE_SANITIZERS "Enable address and undefined behavior sanitizers" ON)

set(SOURCES
    main.cpp
    mapper/src/mapper.cpp
    mapper/src/median_filtering.cpp
    mapper/src/process1_live.cpp
    mapper/src/utils.cpp
    mapper/src/mapper_emvs_stereo.cpp
    mapper/src/pointcloud_processing.cpp
    src/camera.cpp
    tracker/src/tracker.cpp
    tracker/src/lk_se3.cpp
    talking_test/src/class1.cpp
    talking_test/src/class2.cpp
    talking_test/src/coordinator.cpp
    talking_test/src/common.cpp
    udp/src/udp.cpp
)

add_executable(my_executable ${SOURCES})

if(ENABLE_SANITIZERS)
  	message(STATUS "Building with AddressSanitizer and UndefinedBehaviorSanitizer")
	target_compile_options(my_executable PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  	target_link_options(my_executable PRIVATE -fsanitize=address,undefined)
endif()

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/udp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mapper/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracker/include
    ${CMAKE_CURRENT_SOURCE_DIR}/talking_test/include
    ${CMAKE_SOURCE_DIR}/external/minkindr/include
)
target_link_libraries(my_executable PRIVATE
    cartesian3dgrid
    tf2
    deps
)