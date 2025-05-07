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
    udp/src/udp.cpp
    depth_perception/src/depth_perception.cpp
    depth_perception/src/robot_handler.cpp
)

add_executable(my_executable ${SOURCES})

include(CompilerSettings)

if(ENABLE_SANITIZERS)
  	message(STATUS "Building with AddressSanitizer and UndefinedBehaviorSanitizer")
	target_compile_options(my_executable PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  	target_link_options(my_executable PRIVATE -fsanitize=address,undefined)

  # Set flags in one place
  set(SAN_FLAGS
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
      -fno-optimize-sibling-calls
      -g3
      -O1
  )

  target_compile_options(my_executable PRIVATE ${SAN_FLAGS})
  target_link_options(my_executable PRIVATE ${SAN_FLAGS} -rdynamic)

  # Ensure LeakSanitizer runs under ASan
  set(ENV{ASAN_OPTIONS} "symbolize=1:verbosity=2:halt_on_error=1:detect_stack_use_after_return=1:malloc_context_size=20")
  set(ENV{UBSAN_OPTIONS} "print_stacktrace=1:halt_on_error=1")

endif()

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/udp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mapper/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracker/include
    ${CMAKE_CURRENT_SOURCE_DIR}/depth_perception/include
    ${CMAKE_SOURCE_DIR}/external/minkindr/include
    ${CMAKE_SOURCE_DIR}/external/unitree_legged_sdk/include
)
target_link_libraries(my_executable PRIVATE
    cartesian3dgrid
    tf2
    deps
)