file(GLOB SOURCES "src/mapper/*.cpp" "src/tracker/*.cpp" "main.cpp")

message(STATUS "SOURCES: ${SOURCES}")

add_executable(my_executable ${SOURCES})

target_include_directories(my_executable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(my_executable PRIVATE ${OpenCV_LIBS} cartesian3dgrid)