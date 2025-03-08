cmake_minimum_required(VERSION 3.10)
project(MyProject)

include(FetchContent)

set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3party/Eigen_src")
set(OpenCV_VERSION 4.6.0)  # Change to the version you want

# If EIGEN3_INCLUDE_DIR is not defined in the environment, donwload it
if (NOT IS_DIRECTORY ${EIGEN3_INCLUDE_DIR})
    file(MAKE_DIRECTORY ${EIGEN3_INCLUDE_DIR})

    # Declare Eigen as a dependency
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0  # Specify the Eigen version you want
    )

    FetchContent_GetProperties(eigen)
    if (NOT eigen_POPULATED)
        FetchContent_Populate(eigen)

        # Copy the fetched Eigen source to the desired location
        file(COPY ${eigen_SOURCE_DIR}/Eigen DESTINATION ${EIGEN3_INCLUDE_DIR})

    endif()

endif()



FetchContent_Declare(
        opencv
        GIT_REPOSITORY https://github.com/opencv/opencv.git
        GIT_TAG 4.6.0
        #SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-src
)

set(OpenCV_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-build)
set(OpenCV_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/opencv_src)
message(STATUS "OpenCV_INSTALL_DIR: ${OpenCV_INSTALL_DIR}")

#FetchContent_MakeAvailable(opencv)

# Ensure OpenCV is built and installed inside the project
if(NOT EXISTS ${OpenCV_INSTALL_DIR})
    file(MAKE_DIRECTORY ${OpenCV_INSTALL_DIR})
    message(STATUS "Building and installing OpenCV...")

    # Configure OpenCV
    # execute_process(
    #     COMMAND ${CMAKE_COMMAND}
    #         -S ${CMAKE_BINARY_DIR}/_deps/opencv-src
    #         -B ${OpenCV_BUILD_DIR}
    #         -DCMAKE_INSTALL_PREFIX=${OpenCV_INSTALL_DIR}
    #         -DBUILD_SHARED_LIBS=ON
    #         -DBUILD_TESTS=OFF
    #         -DBUILD_EXAMPLES=OFF
    #         -DWITH_IPP=OFF
    #         -DWITH_TBB=ON
    # )

    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/opencv-src
            -B ${OpenCV_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${OpenCV_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=ON
            -DBUILD_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DWITH_IPP=OFF
            -DWITH_TBB=ON
            -DBUILD_LIST=core,imgproc
    )

    # Build OpenCV
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${OpenCV_BUILD_DIR})

    # Install OpenCV
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${OpenCV_BUILD_DIR})
endif()

# Set OpenCV_DIR dynamically so CMake can find OpenCV
set(OpenCV_DIR ${OpenCV_INSTALL_DIR}/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

# Create the executable
add_executable(my_executable src/main.cpp)

# Link OpenCV to the executable
target_include_directories(my_executable PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_executable PRIVATE ${OpenCV_LIBS})
# Link Eigen to your target (this should happen after executable definition)
target_include_directories(my_executable PRIVATE ${EIGEN3_INCLUDE_DIR})