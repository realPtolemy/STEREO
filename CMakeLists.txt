cmake_minimum_required(VERSION 3.5)
project(MyProject)

include(FetchContent)

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/3party)

# ================ LZ4 ================
set(LZ4_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/lz4_src)
message(STATUS "Checking for LZ4 at: ${LZ4_INSTALL_DIR}")

if(NOT EXISTS ${LZ4_INSTALL_DIR})
    message(STATUS "LZ4 not found at ${LZ4_INSTALL_DIR}")
    message(STATUS "Fetching and building LZ4")
    
    FetchContent_Declare(
        lz4
        GIT_REPOSITORY https://github.com/lz4/lz4.git
        GIT_TAG v1.9.4
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/lz4-src
    )
    
    message(STATUS "Declared LZ4 fetch content with tag v1.9.4")
    message(STATUS "Making LZ4 content available")
    FetchContent_MakeAvailable(lz4)
    
    message(STATUS "Building LZ4 using Makefile in: ${CMAKE_BINARY_DIR}/_deps/lz4-src")
    execute_process(
        COMMAND make
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/lz4-src
        RESULT_VARIABLE BUILD_RESULT
    )
    if(BUILD_RESULT EQUAL 0)
        message(STATUS "LZ4 build successful")
    else()
        message(FATAL_ERROR "LZ4 build failed with result: ${BUILD_RESULT}")
    endif()

    message(STATUS "Installing LZ4 to: ${LZ4_INSTALL_DIR}")
    execute_process(
        COMMAND make install PREFIX=${LZ4_INSTALL_DIR}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/lz4-src
        RESULT_VARIABLE INSTALL_RESULT
    )
    if(INSTALL_RESULT EQUAL 0)
        message(STATUS "LZ4 installation successful")
    else()
        message(FATAL_ERROR "LZ4 installation failed with result: ${INSTALL_RESULT}")
    endif()
    
    message(STATUS "LZ4 processing complete")
else()
    message(STATUS "LZ4 already exists at ${LZ4_INSTALL_DIR}, skipping fetch and build")
endif()

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
        FetchContent_MakeAvailable(zlib)
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
        FetchContent_MakeAvailable(eigen)
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
       GIT_TAG 4.9.0
       SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-src
   )
  FetchContent_GetProperties(opencv)
  if(NOT opencv_POPULATED)
      FetchContent_MakeAvailable(opencv)
      set(ZLIB_INCLUDE_DIR ${ZLIB_INSTALL_DIR}/include)
      set(ZLIB_LIBRARY ${ZLIB_INSTALL_DIR}/lib/libz.a)
      execute_process(
          COMMAND ${CMAKE_COMMAND}
          -S ${CMAKE_BINARY_DIR}/_deps/opencv-src
          -B ${OpenCV_BUILD_DIR}
          -DCMAKE_INSTALL_PREFIX=${OpenCV_INSTALL_DIR}
          -DBUILD_ZLIB=OFF
          -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
           -DZLIB_INCLUDE_DIR=${ZLIB_INCLUDE_DIR}
           -DZLIB_LIBRARY=${ZLIB_LIBRARY}
           -DBUILD_OPENEXR=ON
           -DBUILD_SHARED_LIBS=OFF
           -DWITH_JPEG=OFF
           -DBUILD_TESTS=OFF
           -DBUILD_EXAMPLES=OFF
           -DWITH_IPP=OFF
           -DWITH_QUIRC=OFF
           -DWITH_GAPI=OFF
           -DWITH_ADE=OFF
           -DWITH_TBB=ON # Remove this and build OpenEXR prior to OpenCV?
           -DBUILD_TBB=ON
           -DBUILD_LIST=core,imgproc,highgui,calib3d,photo
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

# Always fetch and build Boost from source, ignoring local installations
message(STATUS "Fetching Boost 1.87.0...")
file(MAKE_DIRECTORY ${BOOST_INSTALL_DIR})

include(FetchContent)
FetchContent_Declare(
    boost
    GIT_REPOSITORY https://github.com/boostorg/boost.git
    GIT_TAG boost-1.87.0
    SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/boost-src
)
FetchContent_GetProperties(boost)
if(NOT boost_POPULATED)
    message(STATUS "Populating Boost content...")
    FetchContent_Populate(boost)

    # Configure Boost
    message(STATUS "Configuring Boost...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/boost-src
        -B ${BOOST_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${BOOST_INSTALL_DIR}
        -DBOOST_INCLUDE_LIBRARIES=filesystem;system;iostreams
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_CXX_FLAGS="-D_FILE_OFFSET_BITS=64 -D_XOPEN_SOURCE=700 -D_GNU_SOURCE"
        OUTPUT_VARIABLE BOOST_CONFIGURE_OUTPUT
        ERROR_VARIABLE BOOST_CONFIGURE_ERROR
        RESULT_VARIABLE BOOST_CONFIGURE_RESULT
    )
    message(STATUS "Boost configure output: ${BOOST_CONFIGURE_OUTPUT}")
    if(NOT BOOST_CONFIGURE_RESULT EQUAL 0)
        message(STATUS "Boost configure error output: ${BOOST_CONFIGURE_ERROR}")
        message(FATAL_ERROR "Boost configuration failed with result: ${BOOST_CONFIGURE_RESULT}")
    endif()

    # Build Boost
    message(STATUS "Building Boost...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build ${BOOST_BUILD_DIR}
        OUTPUT_VARIABLE BOOST_BUILD_OUTPUT
        ERROR_VARIABLE BOOST_BUILD_ERROR
        RESULT_VARIABLE BOOST_BUILD_RESULT
    )
    message(STATUS "Boost build output: ${BOOST_BUILD_OUTPUT}")
    if(NOT BOOST_BUILD_RESULT EQUAL 0)
        message(STATUS "Boost build error output: ${BOOST_BUILD_ERROR}")
        message(FATAL_ERROR "Boost build failed with result: ${BOOST_BUILD_RESULT}")
    endif()

    # Install Boost
    message(STATUS "Installing Boost...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --install ${BOOST_BUILD_DIR}
        OUTPUT_VARIABLE BOOST_INSTALL_OUTPUT
        ERROR_VARIABLE BOOST_INSTALL_ERROR
        RESULT_VARIABLE BOOST_INSTALL_RESULT
    )
    message(STATUS "Boost install output: ${BOOST_INSTALL_OUTPUT}")
    if(NOT BOOST_INSTALL_RESULT EQUAL 0)
        message(STATUS "Boost install error output: ${BOOST_INSTALL_ERROR}")
        message(FATAL_ERROR "Boost installation failed with result: ${BOOST_INSTALL_RESULT}")
    endif()
endif()

# Ensure CMake uses the freshly built Boost
set(Boost_NO_BOOST_CMAKE TRUE)  # Prevent CMake from using Boost's own CMake config
find_package(Boost REQUIRED COMPONENTS filesystem system iostreams PATHS ${BOOST_INSTALL_DIR} NO_DEFAULT_PATH)

# ================ FLANN ================
set(FLANN_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/flann_src)
set(FLANN_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/flann-build)

if(NOT EXISTS ${FLANN_INSTALL_DIR})
    message(STATUS "Fetching FLANN...")
    file(MAKE_DIRECTORY ${FLANN_INSTALL_DIR})

    # Fetch FLANN source
    include(FetchContent)
    FetchContent_Declare(
        flann
        GIT_REPOSITORY https://github.com/flann-lib/flann.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/flann-src
    )
    FetchContent_GetProperties(flann)
    if(NOT flann_POPULATED)
        message(STATUS "Populating FLANN content...")
        FetchContent_Populate(flann)
    endif()

    # Patch FLANN's root CMakeLists.txt
    message(STATUS "Patching FLANN root CMakeLists.txt...")
    set(FLANN_CMAKELISTS "${CMAKE_BINARY_DIR}/_deps/flann-src/CMakeLists.txt")
    file(READ ${FLANN_CMAKELISTS} FLANN_CMAKELISTS_CONTENT)
    string(REPLACE
        "cmake_minimum_required(VERSION 2.8.12)"
        "cmake_minimum_required(VERSION 3.16.3)"
        FLANN_CMAKELISTS_CONTENT_MODIFIED
        ${FLANN_CMAKELISTS_CONTENT}
    )
    file(WRITE ${FLANN_CMAKELISTS} "${FLANN_CMAKELISTS_CONTENT_MODIFIED}")
    message(STATUS "Root patch applied successfully")

    # Patch FLANN's src/matlab/CMakeLists.txt
    message(STATUS "Patching FLANN src/matlab/CMakeLists.txt...")
    set(FLANN_MATLAB_CMAKELISTS "${CMAKE_BINARY_DIR}/_deps/flann-src/src/matlab/CMakeLists.txt")
    file(READ ${FLANN_MATLAB_CMAKELISTS} FLANN_MATLAB_CONTENT)
    string(REPLACE
        "get_property(FLANN_LIB_LOCATION TARGET flann_s PROPERTY LOCATION)\nget_filename_component(FLANN_LIB_PATH \${FLANN_LIB_LOCATION} PATH)"
        "set(FLANN_LIB_PATH \"\$<TARGET_FILE_DIR:flann_s>\")"
        FLANN_MATLAB_CONTENT_MODIFIED
        ${FLANN_MATLAB_CONTENT}
    )
    file(WRITE ${FLANN_MATLAB_CMAKELISTS} "${FLANN_MATLAB_CONTENT_MODIFIED}")
    message(STATUS "MATLAB patch applied successfully")

    # Configure FLANN
    message(STATUS "Configuring FLANN...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/flann-src
        -B ${FLANN_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${FLANN_INSTALL_DIR}
        -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=None
        -DWITH_LZ4=ON
        -DBUILD_STATIC_LIBS=ON
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_TESTS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_MATLAB_BINDINGS=ON  # Enable MATLAB bindings
        OUTPUT_VARIABLE FLANN_CONFIGURE_OUTPUT
        ERROR_VARIABLE FLANN_CONFIGURE_ERROR
        RESULT_VARIABLE FLANN_CONFIGURE_RESULT
    )
    message(STATUS "FLANN configure output: ${FLANN_CONFIGURE_OUTPUT}")
    if(NOT FLANN_CONFIGURE_RESULT EQUAL 0)
        message(STATUS "FLANN configure error output: ${FLANN_CONFIGURE_ERROR}")
        message(FATAL_ERROR "FLANN configuration failed with result: ${FLANN_CONFIGURE_RESULT}")
    endif()

    # Build FLANN
    message(STATUS "Building FLANN...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build ${FLANN_BUILD_DIR}
        OUTPUT_VARIABLE FLANN_BUILD_OUTPUT
        ERROR_VARIABLE FLANN_BUILD_ERROR
        RESULT_VARIABLE FLANN_BUILD_RESULT
    )
    message(STATUS "FLANN build output: ${FLANN_BUILD_OUTPUT}")
    if(NOT FLANN_BUILD_RESULT EQUAL 0)
        message(STATUS "FLANN build error output: ${FLANN_BUILD_ERROR}")
        message(FATAL_ERROR "FLANN build failed with result: ${FLANN_BUILD_RESULT}")
    endif()

    # Install FLANN
    message(STATUS "Installing FLANN...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --install ${FLANN_BUILD_DIR}
        OUTPUT_VARIABLE FLANN_INSTALL_OUTPUT
        ERROR_VARIABLE FLANN_INSTALL_ERROR
        RESULT_VARIABLE FLANN_INSTALL_RESULT
    )
    message(STATUS "FLANN install output: ${FLANN_INSTALL_OUTPUT}")
    if(NOT FLANN_INSTALL_RESULT EQUAL 0)
        message(STATUS "FLANN install error output: ${FLANN_INSTALL_ERROR}")
        message(FATAL_ERROR "FLANN installation failed with result: ${FLANN_INSTALL_RESULT}")
    endif()
endif()

# Set FLANN paths for later use
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

set(Eigen3_DIR "${CMAKE_SOURCE_DIR}/3party/Eigen_src/share/eigen3/cmake")
find_package(Eigen3 3.4.0 REQUIRED)
message(STATUS "${FLANN_INSTALL_DIR}")

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
        FetchContent_MakeAvailable(pcl)
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/pcl-src
            -B ${PCL_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${PCL_INSTALL_DIR}
            -DEigen3_DIR=${EIGEN3_INSTALL_DIR}/share/eigen3/cmake
            # -DFLANN_INCLUDE_DIR=${FLANN_INSTALL_DIR}/lib/cmake/flann
            # -DFLANN_INCLUDE_DIR=${FLANN_INSTALL_DIR}/include
            # -DFLANN_LIBRARY=${FLANN_INSTALL_DIR}/lib/libflann_cpp.so
            -DFLANN_ROOT=${FLANN_INSTALL_DIR}
            -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
            # -DBOOST_ROOT=${BOOST_INSTALL_DIR}                  # REMOVE?
           # "-DCMAKE_SHARED_LINKER_FLAGS=-L/opt/homebrew/lib -llz4"  # Link flag, important - without this PCL builder wont find LZ4 lib... not good solution...
            #"-DCMAKE_SHARED_LINKER_FLAGS=${CMAKE_CURRENT_SORUCE_DIR}/3party/lz4_src/lib/liblz4.a"
            "-DCMAKE_SHARED_LINKER_FLAGS=${LZ4_LIBRARY_PATH}"  # Full path here
            -DBUILD_SHARED_LIBS=OFF
            # -DBUILD_octree=OFF                              # REMOVE?
            -DBUILD_filters=ON
            -DBUILD_keypoints=ON
            # -DBUILD_sample_consensus=OFF
            -DBUILD_registration=OFF
            -DBUILD_surface=OFF
            -DBUILD_visualization=OFF
            -DBUILD_global_tests=OFF
            -DBUILD_ml=OFF
            -DBUILD_tools=OFF
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
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${PCL_BUILD_DIR})
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${PCL_BUILD_DIR})
    endif()
endif()
list(APPEND CMAKE_PREFIX_PATH ${PCL_INSTALL_DIR})
find_package(PCL REQUIRED)


# # ================ KINDR ================
# set(KINDR_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/kindr-build)
# set(KINDR_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/kindr_src)
# if(NOT EXISTS ${KINDR_INSTALL_DIR})
#     message(STATUS "Fetching Kindr...")
#     file(MAKE_DIRECTORY ${KINDR_INSTALL_DIR})
#     FetchContent_Declare(
#         kindr
#         GIT_REPOSITORY https://github.com/ANYbotics/kindr.git
#         GIT_TAG master
#         SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/kindr-src
#     )
#     FetchContent_GetProperties(kindr)
#     if (NOT kindr_POPULATED)
#         FetchContent_Populate(kindr)
#     endif()
#     message(STATUS "Building and installing Kindr...")
#     execute_process(
#         COMMAND ${CMAKE_COMMAND}
#             -S ${CMAKE_BINARY_DIR}/_deps/kindr-src
#             -B ${KINDR_BUILD_DIR}
#             -DCMAKE_INSTALL_PREFIX=${KINDR_INSTALL_DIR}
#             -DEIGEN3_INCLUDE_DIR=${EIGEN3_INSTALL_DIR}/include/eigen3
#             -DUSE_CMAKE=ON
#             -DBUILD_TEST=OFF
#             -DCMAKE_BUILD_TYPE=None
#     )
#     execute_process(COMMAND ${CMAKE_COMMAND} --build ${KINDR_BUILD_DIR})
#     execute_process(COMMAND ${CMAKE_COMMAND} --install ${KINDR_BUILD_DIR})
# endif()
# set(kindr_DIR ${KINDR_INSTALL_DIR}/share/kindr/cmake)
# find_package(kindr REQUIRED)

# ================ CNPY ================
set(CNPY_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/cnpy_src)
set(CNPY_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/cnpy-build)
message(STATUS "Checking for CNPY at: ${CNPY_INSTALL_DIR}")

if(NOT EXISTS ${CNPY_INSTALL_DIR})
    message(STATUS "CNPY not found at ${CNPY_INSTALL_DIR}")
    message(STATUS "Fetching CNPY...")

    FetchContent_Declare(
        cnpy
        GIT_REPOSITORY https://github.com/rogersce/cnpy.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/cnpy-src
    )
    message(STATUS "Declared CNPY fetch content from master branch")

    FetchContent_GetProperties(cnpy)
    if(NOT cnpy_POPULATED)
        message(STATUS "CNPY content not populated, starting population process")
        FetchContent_MakeAvailable(cnpy)
        message(STATUS "CNPY populated to: ${CMAKE_BINARY_DIR}/_deps/cnpy-src")
    else()
        message(STATUS "CNPY content already populated at: ${CMAKE_BINARY_DIR}/_deps/cnpy-src")
    endif()

    message(STATUS "Configuring CNPY from source: ${CMAKE_BINARY_DIR}/_deps/cnpy-src")
    message(STATUS "Using ZLIB_ROOT: ${ZLIB_ROOT}")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/cnpy-src
        -B ${CNPY_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${CNPY_INSTALL_DIR}
        -DBUILD_SHARED_LIBS=OFF
        -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
        RESULT_VARIABLE CONFIG_RESULT
    )
    if(CONFIG_RESULT EQUAL 0)
        message(STATUS "CNPY configuration successful")
    else()
        message(FATAL_ERROR "CNPY configuration failed with result: ${CONFIG_RESULT}")
    endif()

    message(STATUS "Building CNPY in: ${CNPY_BUILD_DIR}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build ${CNPY_BUILD_DIR}
        RESULT_VARIABLE BUILD_RESULT
    )
    if(BUILD_RESULT EQUAL 0)
        message(STATUS "CNPY build successful")
    else()
        message(FATAL_ERROR "CNPY build failed with result: ${BUILD_RESULT}")
    endif()

    message(STATUS "Installing CNPY to: ${CNPY_INSTALL_DIR}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --install ${CNPY_BUILD_DIR}
        RESULT_VARIABLE INSTALL_RESULT
    )
    if(INSTALL_RESULT EQUAL 0)
        message(STATUS "CNPY installation successful")
    else()
        message(FATAL_ERROR "CNPY installation failed with result: ${INSTALL_RESULT}")
    endif()

    message(STATUS "CNPY processing complete")
else()
    message(STATUS "CNPY already exists at ${CNPY_INSTALL_DIR}, skipping fetch and build")
endif()

# ================ Sophus ================
set(SOPHUS_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/sophus_src)
set(SOPHUS_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/sophus-build)
if(NOT EXISTS ${SOPHUS_INSTALL_DIR})
    message(STATUS "Fetching Sophus...")
    FetchContent_Declare(
        sophus
        GIT_REPOSITORY https://github.com/strasdat/Sophus.git
        GIT_TAG 1.24.6
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/sophus-src
    )
    FetchContent_GetProperties(sophus)
    if(NOT sophus_POPULATED)
        FetchContent_MakeAvailable(sophus)
    endif()
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/sophus-src
        -B ${SOPHUS_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${SOPHUS_INSTALL_DIR}
        -DEigen3_DIR=${Eigen3_DIR}
        -DBUILD_SOPHUS_TESTS=OFF
    )
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${SOPHUS_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${SOPHUS_BUILD_DIR})
endif()
# find_package(Sophus REQUIRED)

# ================ Subdirectories ================
# add_subdirectory(minkindr)
# add_subdirectory(tf2)
# add_subdirectory(cartesian3dgrid)
add_subdirectory(external)
add_subdirectory(app)