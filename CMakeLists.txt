cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3party/Eigen_src")

# If EIGEN3_INCLUDE_DIR is not defined in the environment, donwload it
if (NOT IS_DIRECTORY ${EIGEN3_INCLUDE_DIR})
    include(FetchContent)

    # Declare Eigen as a dependency
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0  # Specify the Eigen version you want
    )

    # Fetch Eigen if not already available
    FetchContent_MakeAvailable(eigen)

    # Add the include directory for the target
    include_directories(${eigen_SOURCE_DIR})
endif()

# Add your executable target towards the end
add_executable(my_executable src/main.cpp)

# Link Eigen to your target (this should happen after executable definition)
target_include_directories(my_executable PRIVATE ${EIGEN3_INCLUDE_DIR})