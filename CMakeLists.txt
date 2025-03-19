cmake_minimum_required(VERSION 3.14)
project(MyProject)

include(FetchContent)

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/3party)

# ================ Eigen ================
set(EIGEN3_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/Eigen_src)
set(EIGEN3_BUILD_DIR ${CMAKE_SOURCE_DIR}/3party/opencv_src)
if (NOT IS_DIRECTORY ${EIGEN3_INSTALL_DIR})
    message(STATUS "Fetching Eigen...")
    file(MAKE_DIRECTORY ${EIGEN3_INSTALL_DIR})

    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
    )

    FetchContent_GetProperties(eigen)
    if (NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${eigen_SOURCE_DIR}
            -B ${CMAKE_BINARY_DIR}/_deps/eigen-build
            -DCMAKE_INSTALL_PREFIX=${EIGEN3_INSTALL_DIR}
            -DBUILD_TESTING=OFF
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}/_deps/eigen-build)
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR}/_deps/eigen-build)

    endif()
    message(STATUS "Eigen fetched and installed.")
endif()

# ================ Protobuf ================
find_package(Protobuf QUIET)
if(NOT Protobuf_FOUND)
    message(STATUS "Protobuf not found. Attempting to install it...")
    if(UNIX AND NOT APPLE)  # Linux
        execute_process(COMMAND sudo apt update)
        execute_process(COMMAND sudo apt install -y libprotobuf-dev protobuf-compiler)
    elseif(APPLE)  # macOS
        execute_process(COMMAND brew install protobuf)
    elseif(WIN32)  # Windows
        execute_process(COMMAND vcpkg install protobuf)
        set(Protobuf_DIR "$ENV{VCPKG_ROOT}/installed/x64-windows/share/protobuf")
    else()
        message(FATAL_ERROR "Unsupported OS. Install Protobuf manually.")
    endif()
    find_package(Protobuf REQUIRED)
    if(NOT Protobuf_FOUND)
        message(FATAL_ERROR "Failed to install Protobuf.")
    endif()
endif()
message(STATUS "Protobuf found: ${Protobuf_LIBRARIES}, ${Protobuf_INCLUDE_DIRS}, Protoc: ${Protobuf_PROTOC_EXECUTABLE}")

# ================ OpenCV ================
set(OpenCV_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-build)
set(OpenCV_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/opencv_src)
set(OpenCV_VERSION 4.6.0)
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
            -DWITH_QUIRC=OFF
            -DWITH_TBB=ON
            -DBUILD_LIST=core,imgproc,highgui
            -DWITH_PROTOBUF=ON
            -DProtobuf_LIBRARY=${Protobuf_LIBRARIES}
            -DProtobuf_INCLUDE_DIR=${Protobuf_INCLUDE_DIRS}
            -DProtobuf_PROTOC_EXECUTABLE=${Protobuf_PROTOC_EXECUTABLE}
            # Ensure OpenCV doesn't build its own Protobuf
            -DBUILD_opencv_python2=OFF
            -DBUILD_opencv_python3=OFF
            -DBUILD_PROTOBUF=OFF
    )

    # Build OpenCV
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${OpenCV_BUILD_DIR})

    # Install OpenCV
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${OpenCV_BUILD_DIR})
endif()

# ================ PCL DEPENDECIES ================
# ================ Boost ================
find_package(Boost REQUIRED COMPONENTS filesystem system iostreams)

# If Boost is not found, attempt to install it
if (NOT Boost_FOUND)
    message(STATUS "Boost not found. Attempting to install required components...")

    if (UNIX AND NOT APPLE)  # Linux (Ubuntu, Debian, Fedora, Arch)
        execute_process(COMMAND sudo apt update)  # Ubuntu/Debian
        execute_process(COMMAND sudo apt install -y libboost-filesystem-dev libboost-system-dev libboost-iostreams-dev)  # Ubuntu/Debian
        execute_process(COMMAND sudo dnf install -y boost-filesystem boost-system boost-iostreams)  # Fedora
        execute_process(COMMAND sudo pacman -S --noconfirm boost-libs)  # Arch Linux
    elseif (APPLE)  # macOS (Homebrew)
        execute_process(COMMAND brew install boost-filesystem boost-system boost-iostreams)
    elseif (WIN32)  # Windows (Vcpkg/MSYS2)
        execute_process(COMMAND vcpkg install boost-filesystem boost-system boost-iostreams)
        set(Boost_DIR "$ENV{VCPKG_ROOT}/installed/x64-windows/share/boost")
    else()
        message(FATAL_ERROR "Unsupported operating system. Please install Boost manually.")
    endif()

    # Re-run find_package(Boost) after installation
    find_package(Boost REQUIRED COMPONENTS filesystem system iostreams)

    if (NOT Boost_FOUND)
        message(FATAL_ERROR "Failed to install Boost. Please install it manually.")
    endif()
endif()
message(STATUS "Boost successfully found at: ${Boost_INCLUDE_DIRS}")

# ==================== Ensure pkg-config is Installed ====================
find_program(PKG_CONFIG_EXECUTABLE pkg-config)

if(NOT PKG_CONFIG_EXECUTABLE)
    message(STATUS "pkg-config not found. Attempting to install it...")

    if(UNIX AND NOT APPLE)  # Linux (Ubuntu, Debian, Fedora, Arch)
        execute_process(COMMAND sudo apt update)  # Ubuntu/Debian
        execute_process(COMMAND sudo apt install -y pkg-config)  # Ubuntu/Debian
        execute_process(COMMAND sudo dnf install -y pkgconf)  # Fedora
        execute_process(COMMAND sudo pacman -S --noconfirm pkgconf)  # Arch Linux
    elseif(APPLE)  # macOS (Homebrew)
        execute_process(COMMAND brew install pkg-config)
    elseif(WIN32)  # Windows (MSYS2)
        execute_process(COMMAND pacman -S --noconfirm mingw-w64-x86_64-pkg-config)
    else()
        message(FATAL_ERROR "Unsupported operating system. Please install pkg-config manually.")
    endif()

    find_program(PKG_CONFIG_EXECUTABLE pkg-config)  # Check again after installation

    if(NOT PKG_CONFIG_EXECUTABLE)
        message(FATAL_ERROR "Failed to install pkg-config. Please install it manually.")
    endif()
endif()

# ================ LZ4 ================
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LZ4 QUIET liblz4)
    if(LZ4_FOUND)
        message(STATUS "LZ4 via pkg-config: ${LZ4_LIBRARIES}, ${LZ4_INCLUDE_DIRS}")
        # Ensure full path for linking
        find_library(LZ4_FULL_PATH NAMES lz4 liblz4 HINTS ${LZ4_LIBRARY_DIRS} /opt/homebrew/lib)
        if(LZ4_FULL_PATH)
            set(LZ4_LIBRARIES ${LZ4_FULL_PATH})
        endif()
    endif()
endif()

if(NOT LZ4_FOUND)
    find_path(LZ4_INCLUDE_DIR lz4.h HINTS /opt/homebrew/include /usr/local/include /usr/include)
    find_library(LZ4_LIBRARY NAMES lz4 liblz4 HINTS /opt/homebrew/lib /usr/local/lib /usr/lib)
    if(LZ4_INCLUDE_DIR AND LZ4_LIBRARY)
        set(LZ4_LIBRARIES ${LZ4_LIBRARY})
        set(LZ4_INCLUDE_DIRS ${LZ4_INCLUDE_DIR})
        set(LZ4_FOUND TRUE)
    endif()
endif()

if(NOT LZ4_FOUND)
    message(STATUS "LZ4 not found. Installing via Homebrew...")
    execute_process(COMMAND brew install lz4)
    # Retry detection
    find_path(LZ4_INCLUDE_DIR lz4.h HINTS /opt/homebrew/include)
    find_library(LZ4_LIBRARY NAMES lz4 liblz4 HINTS /opt/homebrew/lib)
    if(LZ4_INCLUDE_DIR AND LZ4_LIBRARY)
        set(LZ4_LIBRARIES ${LZ4_LIBRARY})
        set(LZ4_INCLUDE_DIRS ${LZ4_INCLUDE_DIR})
        set(LZ4_FOUND TRUE)
    endif()
endif()

if(LZ4_FOUND)
    message(STATUS "LZ4 final config: Libraries=${LZ4_LIBRARIES}, Includes=${LZ4_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "LZ4 not found after installation attempts.")
endif()

#================ FLANN ================
set(FLANN_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/flann-build)
set(FLANN_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/flann_src)

if(NOT EXISTS ${FLANN_INSTALL_DIR})
    message(STATUS "Fetching FLANN...")
    file(MAKE_DIRECTORY ${FLANN_INSTALL_DIR})
    FetchContent_Declare(
        flann
        GIT_REPOSITORY https://github.com/flann-lib/flann.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/flann-src
    )
    FetchContent_GetProperties(flann)
    if (NOT flann_POPULATED)
        FetchContent_Populate(flann)
    endif()
    message(STATUS "Building and installing FLANN...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/flann-src
            -B ${FLANN_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${FLANN_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_STATIC_LIBS=ON
            -DBUILD_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DCMAKE_BUILD_TYPE=None
            -DWITH_LZ4=ON # Force add LZ4 to install, required by PCL library
    )
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${FLANN_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${FLANN_BUILD_DIR})
endif()

# # Ensure CMake knows where FLANN is installed
set(FLANN_DIR ${FLANN_INSTALL_DIR}/lib/cmake/flann)
set(FLANN_INCLUDE_DIR ${FLANN_INSTALL_DIR}/include)
# set(FLANN_LIBRARY ${FLANN_INSTALL_DIR}/lib/libflann_cpp.a)

set(ENV{PKG_CONFIG_PATH} "${FLANN_INSTALL_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -llz4")
# target_link_libraries(pcl_compute_hausdorff lz4)

# ================ PCL ================
set(PCL_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-build)
set(PCL_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/pcl_src)
message(STATUS "PCL_INSTALL_DIR: ${PCL_INSTALL_DIR}")

if(NOT EXISTS ${PCL_INSTALL_DIR})
    file(MAKE_DIRECTORY ${PCL_INSTALL_DIR})
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
    message(STATUS "Building and installing PCL...")
    message(STATUS "LZ4_LIBRARIES: ${LZ4_LIBRARIES}") # Print path to LZ4 libs for check
    message(STATUS "CMAKE_SHARED_LINKER_FLAGS: -L/opt/homebrew/lib -llz4") # Print link, if linking is successful, no error
    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/pcl-src
            -B ${PCL_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${PCL_INSTALL_DIR}
            -DEigen3_DIR=${EIGEN3_INSTALL_DIR}/share/eigen3/cmake
            -DFLANN_DIR=${FLANN_INSTALL_DIR}/lib/cmake/flann
            -DLZ4_LIBRARIES=${LZ4_LIBRARIES}
            -DLZ4_INCLUDE_DIRS=${LZ4_INCLUDE_DIRS}
            -DCMAKE_LIBRARY_PATH=${LZ4_LIBRARY_DIRS}
            "-DCMAKE_SHARED_LINKER_FLAGS=-L/opt/homebrew/lib -llz4"  # Link flag, important...
            -DCMAKE_BUILD_TYPE=None
            -DPCL_BUILD_TOOLS=OFF
            # Enable only necessary PCL modules
            -DPCL_BUILD_COMMON=ON
            -DPCL_BUILD_IO=ON
            -DPCL_BUILD_FILTERS=ON
            -DPCL_BUILD_KDTREE=ON
            -DPCL_BUILD_OCTREE=ON
            # Disable VTK and visualization modules
            -DWITH_VTK=OFF
            -DWITH_QT=OFF
            -DWITH_QHULL=OFF
            -DWITH_PNG=OFF
            -DWITH_OPENGL=OFF
            -DWITH_LIBUSB=OFF
            -DWITH_OPENNI=OFF
            -DWITH_PCAP=OFF
            -DWITH_OPENNI2=OFF
            -DWITH_CUDA=OFF
            -DWITH_ENSENSO=OFF
            -DWITH_DAVIDSDK=OFF
            -DWITH_DSSDK=OFF
            -DWITH_RSSDK=OFF
            -DWITH_RSSDK2=OFF
            # Disable other unnecessary modules
            -DPCL_BUILD_APPS=OFF
            -DPCL_BUILD_EXAMPLES=OFF
            -DPCL_BUILD_TESTS=OFF
    )

    execute_process(COMMAND ${CMAKE_COMMAND} --build ${PCL_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${PCL_BUILD_DIR})
endif()

# Set OpenCV_DIR dynamically so CMake can find OpenCV
set(OpenCV_DIR ${OpenCV_INSTALL_DIR}/lib/cmake/opencv4)
# set(PCL_DIR ${PCL_INSTALL_DIR}/lib/cmake/pcl)
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

set(Cnpy_DIR ${Cnpy_INSTALL_DIR}/lib/cmake)

# Behöver ZLIB för att bygga CNPY
add_subdirectory(cartesian3dgrid)
add_subdirectory(app)


# add_executable(my_project main.cpp)
# target_include_directories(my_project PRIVATE ${Cnpy_INSTALL_DIR}/include)
# target_link_libraries(my_project PRIVATE cnpy)