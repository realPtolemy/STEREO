cmake_minimum_required(VERSION 3.20)
project(MyProject)

include(FetchContent)

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/3party)

# ================ Zlib ================
set(ZLIB_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/zlib_src)
set(ZLIB_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/zlib-build)
if(NOT EXISTS ${ZLIB_INSTALL_DIR})
    message(STATUS "Fetching and building zlib...")
    FetchContent_Declare(
        zlib
        GIT_REPOSITORY https://github.com/madler/zlib.git
        GIT_TAG v1.3.1
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/zlib-src
    )
    FetchContent_GetProperties(zlib)
    if(NOT zlib_POPULATED)
        FetchContent_Populate(zlib)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/zlib-src
            -B ${ZLIB_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${ZLIB_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${ZLIB_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${ZLIB_BUILD_DIR})
    endif()
    message(STATUS "Zlib fetched and installed.")
endif()
set(ZLIB_ROOT ${ZLIB_INSTALL_DIR})
find_package(ZLIB REQUIRED)

# ================ Eigen ================
set(EIGEN3_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/Eigen_src)
if(NOT EXISTS ${EIGEN3_INSTALL_DIR})
    message(STATUS "Fetching and building Eigen...")
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/eigen-src
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/eigen-src
            -B ${CMAKE_BINARY_DIR}/_deps/eigen-build
            -DCMAKE_INSTALL_PREFIX=${EIGEN3_INSTALL_DIR}
            -DBUILD_TESTING=OFF
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}/_deps/eigen-build)
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR}/_deps/eigen-build)
    endif()
    message(STATUS "Eigen fetched and installed.")
endif()

# ================ OpenCV ================
set(OpenCV_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/opencv_src)
set(OpenCV_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-build)
if(NOT EXISTS ${OpenCV_INSTALL_DIR})
    message(STATUS "Fetching OpenCV...")
    FetchContent_Declare(
        opencv
        GIT_REPOSITORY https://github.com/opencv/opencv.git
        GIT_TAG 4.6.0
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-src
    )
    FetchContent_GetProperties(opencv)
    if(NOT opencv_POPULATED)
        FetchContent_Populate(opencv)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/opencv-src
            -B ${OpenCV_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${OpenCV_INSTALL_DIR}
            -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
            -DBUILD_OPENEXR=ON
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DWITH_IPP=OFF
            -DWITH_QUIRC=OFF
            -DWITH_GAPI=OFF
            -DWITH_ADE=OFF
            -DWITH_TBB=ON # Remove this and build OpenEXR prior to OpenCV?
            -DBUILD_TBB=ON
            -DBUILD_LIST=core,imgproc,highgui
            -DWITH_PROTOBUF=OFF
            -DBUILD_PROTOBUF=OFF
            -DBUILD_opencv_python2=OFF
            -DBUILD_opencv_python3=OFF
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${OpenCV_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${OpenCV_BUILD_DIR})
    endif()
endif()
set(OpenCV_DIR ${OpenCV_INSTALL_DIR}/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

# ================ Boost ================
set(BOOST_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/boost_src)
set(BOOST_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/boost-build)
if(NOT EXISTS ${BOOST_INSTALL_DIR})
    message(STATUS "Fetching Boost...")
    FetchContent_Declare(
        boost
        GIT_REPOSITORY https://github.com/boostorg/boost.git
        GIT_TAG boost-1.87.0
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/boost-src
    )
    FetchContent_GetProperties(boost)
    if(NOT boost_POPULATED)
        FetchContent_Populate(boost)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/boost-src
            -B ${BOOST_BUILD_DIR}
            #-DBOOST_INCLUDE_LIBRARIES="filesystem;system;iostreams"
            -DBUILD_SHARED_LIBS=OFF
            -DCMAKE_INSTALL_PREFIX=${BOOST_INSTALL_DIR}
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${BOOST_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${BOOST_BUILD_DIR})
    endif()
endif()
set(Boost_NO_BOOST_CMAKE TRUE)
find_package(Boost REQUIRED COMPONENTS filesystem system iostreams PATHS ${BOOST_INSTALL_DIR} NO_DEFAULT_PATH)

# ================ LZ4 ================
set(LZ4_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/lz4_src)
set(LZ4_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/lz4-build)
if(NOT EXISTS ${LZ4_INSTALL_DIR})
    message(STATUS "Fetching LZ4...")
    FetchContent_Declare(
        lz4
        GIT_REPOSITORY https://github.com/lz4/lz4.git
        GIT_TAG v1.9.4
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/lz4-src
    )
    FetchContent_GetProperties(lz4)
    if(NOT lz4_POPULATED)
        FetchContent_Populate(lz4)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/lz4-src/build/cmake
            -B ${LZ4_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${LZ4_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DLZ4_BUILD_CLI=OFF
            -DLZ4_BUILD_LEGACY_LZ4C=OFF
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${LZ4_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${LZ4_BUILD_DIR})
    endif()
endif()
set(LZ4_LIBRARIES ${LZ4_INSTALL_DIR}/lib/liblz4.a)
set(LZ4_INCLUDE_DIRS ${LZ4_INSTALL_DIR}/include)
set(ENV{PKG_CONFIG_PATH} "${LZ4_INSTALL_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# ================ FLANN ================
set(FLANN_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/flann_src)
set(FLANN_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/flann-build)
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
    if(NOT flann_POPULATED)
        FetchContent_Populate(flann)
    endif()
    message(STATUS "Building and installing FLANN...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/flann-src
        -B ${FLANN_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${FLANN_INSTALL_DIR}
        -DZLIB_ROOT=${ZLIB_INSTALL_DIR}                        # REMOVE?
        -DCMAKE_BUILD_TYPE=None
        -DWITH_LZ4=ON                                          # REMOVE?
        -DBUILD_STATIC_LIBS=ON                                 # REMOVE?
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_TESTS=OFF
        -DBUILD_EXAMPLES=OFF
    )
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${FLANN_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${FLANN_BUILD_DIR})
endif()
set(FLANN_DIR ${FLANN_INSTALL_DIR}/lib/cmake/flann)
set(FLANN_INCLUDE_DIR ${FLANN_INSTALL_DIR}/include)
set(ENV{PKG_CONFIG_PATH} "${FLANN_INSTALL_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# ================ PCL ================
set(PCL_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/pcl_src)
set(PCL_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-build)
## START OF EXTRA CHECK that LZ4 can be found because of PCL's problematic dependency on LZ4...
set(LZ4_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/3party/lz4_src/lib/liblz4.a")
if(NOT EXISTS ${LZ4_LIBRARY_PATH})
    message(FATAL_ERROR "LZ4 library not found at ${LZ4_LIBRARY_PATH}")
endif()
## END OF EXTRA CHECK
if(NOT EXISTS ${PCL_INSTALL_DIR})
    message(STATUS "Fetching PCL...")
    FetchContent_Declare(
        pcl
        GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-src
    )
    FetchContent_GetProperties(pcl)
    if(NOT pcl_POPULATED)
        FetchContent_Populate(pcl)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/pcl-src
            -B ${PCL_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${PCL_INSTALL_DIR}
            -DEigen3_DIR=${EIGEN3_INSTALL_DIR}/share/eigen3/cmake
            -DFLANN_DIR=${FLANN_INSTALL_DIR}/lib/cmake/flann   # REMOVE?
            -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
            -DLZ4_LIBRARIES=${LZ4_LIBRARIES}                   # REMOVE?
            -DLZ4_INCLUDE_DIRS=${LZ4_INCLUDE_DIRS}             # REMOVE?
            -DBOOST_ROOT=${BOOST_INSTALL_DIR}                  # REMOVE?
           # "-DCMAKE_SHARED_LINKER_FLAGS=-L/opt/homebrew/lib -llz4"  # Link flag, important - without this PCL builder wont find LZ4 lib... not good solution...
            #"-DCMAKE_SHARED_LINKER_FLAGS=${CMAKE_CURRENT_SORUCE_DIR}/3party/lz4_src/lib/liblz4.a"
            "-DCMAKE_SHARED_LINKER_FLAGS=${LZ4_LIBRARY_PATH}"  # Full path here
            -DBUILD_SHARED_LIBS=OFF
            -DPCL_BUILD_TOOLS=OFF                              # REMOVE?
            -DPCL_BUILD_COMMON=ON                              # REMOVE?
            -DPCL_BUILD_IO=ON                                  # REMOVE?
            -DPCL_BUILD_FILTERS=ON                             # REMOVE?
            -DPCL_BUILD_KDTREE=ON                              # REMOVE?
            -DPCL_BUILD_OCTREE=ON                              # REMOVE?
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
            -DPCL_BUILD_APPS=OFF                               # REMOVE?
            -DPCL_BUILD_EXAMPLES=OFF                           # REMOVE?
            -DPCL_BUILD_TESTS=OFF                              # REMOVE?
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${PCL_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${PCL_BUILD_DIR})
    endif()
endif()
list(APPEND CMAKE_PREFIX_PATH ${PCL_INSTALL_DIR})
find_package(PCL REQUIRED)

# ================ CNPY ================
set(CNPY_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/cnpy_src)
set(CNPY_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/cnpy-build)
if(NOT EXISTS ${CNPY_INSTALL_DIR})
    message(STATUS "Fetching CNPY...")
    FetchContent_Declare(
        cnpy
        GIT_REPOSITORY https://github.com/rogersce/cnpy.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/cnpy-src
    )
    FetchContent_GetProperties(cnpy)
    if(NOT cnpy_POPULATED)
        FetchContent_Populate(cnpy)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/cnpy-src
            -B ${CNPY_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${CNPY_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${CNPY_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${CNPY_BUILD_DIR})
    endif()
endif()

# ================ Subdirectories ================
add_subdirectory(cartesian3dgrid)
add_subdirectory(app)