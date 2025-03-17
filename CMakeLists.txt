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

set(OpenCV_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-build)
set(OpenCV_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/opencv_src)
message(STATUS "OpenCV_INSTALL_DIR: ${OpenCV_INSTALL_DIR}")

# Ensure OpenCV is built and installed inside the project
if(NOT EXISTS ${OpenCV_INSTALL_DIR})
    message(STATUS "Fetching OpenCV...")

    FetchContent_Declare(
            opencv
            GIT_REPOSITORY https://github.com/opencv/opencv.git
            GIT_TAG 4.6.0
            SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-src
    )
    FetchContent_GetProperties(opencv)
    if (NOT opencv_POPULATED)
        FetchContent_Populate(opencv)
    endif()
    file(MAKE_DIRECTORY ${OpenCV_INSTALL_DIR})
    message(STATUS "Building and installing OpenCV...")

    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/opencv-src
            -B ${OpenCV_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${OpenCV_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=off
            -DBUILD_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DWITH_IPP=OFF
            -DWITH_TBB=ON
            -DBUILD_LIST=core,imgproc,highgui
    )

    # Build OpenCV
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${OpenCV_BUILD_DIR})

    # Install OpenCV
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${OpenCV_BUILD_DIR})
endif()

set(PCL_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-build)
set(PCL_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/pcl_src)
message(STATUS "PCL_INSTALL_DIR: ${PCL_INSTALL_DIR}")

if(NOT EXISTS ${PCL_INSTALL_DIR})

    FetchContent_Declare(
        pcl
        GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-src
    )
    FetchContent_GetProperties(pcl)
    if (NOT pcl_POPULATED)
        FetchContent_Populate(pcl)
    endif()
    # file(MAKE_DIRECTORY ${PCL_INSTALL_DIR})
    message(STATUS "Building and installing PCL...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/pcl-src
        -B ${PCL_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${PCL_INSTALL_DIR}
        -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_TESTS=OFF
        -DCMAKE_BUILD_TYPE=None
        -DBUILD_GPU=OFF
        -DBUILD_apps=OFF
        -DBUILD_examples=OFF
    )

    execute_process(COMMAND ${CMAKE_COMMAND} --build ${PCL_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${PCL_BUILD_DIR})
endif()

# Set OpenCV_DIR dynamically so CMake can find OpenCV
set(OpenCV_DIR ${OpenCV_INSTALL_DIR}/lib/cmake/opencv4)
set(PCL_DIR ${PCL_INSTALL_DIR}/lib/cmake/pcl)
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

set(Cnpy_DIR ${Cnpy_INSTALL_DIR}/lib/cmake)

# Behöver ZLIB för att bygga CNPY
add_subdirectory(cartesian3dgrid)
add_subdirectory(app)


# add_executable(my_project main.cpp)
# target_include_directories(my_project PRIVATE ${Cnpy_INSTALL_DIR}/include)
# target_link_libraries(my_project PRIVATE cnpy)