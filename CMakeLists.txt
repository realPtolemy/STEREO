cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0144 NEW)
project(MyProject)


file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/3party)
include(FetchContent)

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
    
    # Clean up
    #message(STATUS "Cleaning up LZ4 build artifacts in ${CMAKE_BINARY_DIR}/_deps/...")
    #execute_process(
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/lz4-src
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/lz4-build
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/lz4-subbuild
    #    RESULT_VARIABLE CLEAN_RESULT
    #)
    #if(NOT CLEAN_RESULT EQUAL 0)
    #    message(WARNING "Failed to clean up LZ4 build artifacts: ${CLEAN_RESULT}")
    #else()
    #    message(STATUS "LZ4 build artifacts cleaned up.")
    #endif()

    message(STATUS "LZ4 processing complete")
else()
    message(STATUS "LZ4 already exists at ${LZ4_INSTALL_DIR}, skipping fetch and build")
endif()
set(LZ4_ROOT ${LZ4_INSTALL_DIR})
message(STATUS "LZ4 can now be found in: ${LZ4_INSTALL_DIR}")

# ================ Zlib ================
set(ZLIB_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/zlib_src)
set(ZLIB_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/zlib-build)
if(NOT EXISTS ${ZLIB_INSTALL_DIR})
    message(STATUS "Fetching and building ZLIB...")
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
    # Clean up
    #message(STATUS "Cleaning up ZLIB build artifacts in ${CMAKE_BINARY_DIR}/_deps/...")
    #execute_process(
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/zlib-src
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/zlib-build
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/zlib-subbuild
    #    RESULT_VARIABLE CLEAN_RESULT
    #)
    #if(NOT CLEAN_RESULT EQUAL 0)
    #    message(WARNING "Failed to clean up ZLIB build artifacts: ${CLEAN_RESULT}")
    #else()
    #    message(STATUS "ZLIB build artifacts cleaned up.")
    #endif()
    message(STATUS "ZLIB fetched and installed.")
endif()
set(ZLIB_ROOT ${ZLIB_INSTALL_DIR})
message(STATUS "ZLIB can now be found in: ${ZLIB_INSTALL_DIR}")

# ================ Eigen ================
set(EIGEN_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/eigen_src)
if(NOT EXISTS ${EIGEN_INSTALL_DIR})
    message(STATUS "Fetching and building EIGEN...")
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
            -DCMAKE_INSTALL_PREFIX=${EIGEN_INSTALL_DIR}
            -DBUILD_TESTING=OFF
        )
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}/_deps/eigen-build)
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR}/_deps/eigen-build)
    endif()

     # Clean up
    #message(STATUS "Cleaning up EIGEN build artifacts in ${CMAKE_BINARY_DIR}/_deps/...")
    #execute_process(
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/eigen-src
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/eigen-build
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/eigen-subbuild
    #    RESULT_VARIABLE CLEAN_RESULT
    #)
    #if(NOT CLEAN_RESULT EQUAL 0)
    #    message(WARNING "Failed to clean up EIGEN build artifacts: ${CLEAN_RESULT}")
    #else()
    #    message(STATUS "EIGEN build artifacts cleaned up.")
    #endif()
    message(STATUS "EIGEN fetched and installed.")
endif()
message(STATUS "Verifying EIGEN installation...")
set(EIGEN_ROOT ${EIGEN_INSTALL_DIR})
message(STATUS "EIGEN can now be found in: ${EIGEN_INSTALL_DIR}")

# ================ Eigen Include Fix ================
find_package(Eigen3 REQUIRED PATHS ${EIGEN_INSTALL_DIR}/share/eigen3/cmake NO_DEFAULT_PATH)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${Eigen3_DIR}")
    include_directories(${EIGEN3_INCLUDE_DIR}) # Global include path
else()
    message(FATAL_ERROR "Eigen3 not found at ${EIGEN_INSTALL_DIR}/share/eigen3/cmake!")
endif()


# ================ OpenCV ================
set(OPENCV_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/opencv_src)
set(OPENCV_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-build)

if(NOT EXISTS ${OPENCV_INSTALL_DIR})
    message(STATUS "Fetching OpenCV...")
    FetchContent_Declare(
        opencv
        GIT_REPOSITORY https://github.com/opencv/opencv.git
        GIT_TAG 4.11.0
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/opencv-src
    )
    FetchContent_GetProperties(opencv)
    if(NOT opencv_POPULATED)
        FetchContent_Populate(opencv)

        # Patch OpenCV's OpenCVGenPkgconfig.cmake to fix cmake_minimum_required issue
        message(STATUS "Patching OpenCVGenPkgconfig.cmake...")
        set(OPENCV_PKGCONFIG_CMAKE "${CMAKE_BINARY_DIR}/_deps/opencv-src/cmake/OpenCVGenPkgconfig.cmake")
        file(READ ${OPENCV_PKGCONFIG_CMAKE} PKGCONFIG_CMAKE_CONTENT)
        string(REPLACE
            "cmake_minimum_required(VERSION 2.8.12.2)"
            "cmake_minimum_required(VERSION 3.5)"
            PKGCONFIG_CMAKE_CONTENT_MODIFIED
            ${PKGCONFIG_CMAKE_CONTENT}
        )
        file(WRITE ${OPENCV_PKGCONFIG_CMAKE} "${PKGCONFIG_CMAKE_CONTENT_MODIFIED}")
        message(STATUS "OpenCVGenPkgconfig.cmake patched successfully")

        set(ZLIB_INCLUDE_DIR ${ZLIB_INSTALL_DIR}/include)
        set(ZLIB_LIBRARY ${ZLIB_INSTALL_DIR}/lib/libz.a)
        message(STATUS "Configuring OpenCV with JPEG support...")

        # Configure OpenCV
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/opencv-src
            -B ${OPENCV_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_ZLIB=OFF
            -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
            -DZLIB_INCLUDE_DIR=${ZLIB_INCLUDE_DIR}
            -DZLIB_LIBRARY=${ZLIB_LIBRARY}
            -DWITH_JPEG=ON
            -DBUILD_JPEG=ON
            -DWITH_TIFF=OFF
            -DBUILD_TIFF=OFF
            -DWITH_PNG=OFF
            -DBUILD_PNG=OFF
            -DWITH_WEBP=OFF
            -DBUILD_WEBP=OFF
            -DWITH_OPENJPEG=OFF
            -DBUILD_OPENJPEG=OFF
            -DWITH_PROTOBUF=OFF
            -DBUILD_PROTOBUF=OFF
            -DWITH_IPP=OFF
            -DWITH_QUIRC=OFF
            -DWITH_GAPI=OFF
            -DWITH_ADE=OFF
            -DWITH_TBB=OFF
            -DBUILD_TBB=OFF
            -DWITH_OPENEXR=OFF
            -DBUILD_OPENEXR=OFF
            -DBUILD_LIST=core,imgproc,highgui,calib3d,photo,imgcodecs
            -DBUILD_opencv_python2=OFF
            -DBUILD_opencv_python3=OFF
            -DBUILD_opencv_stitching=OFF
            -DBUILD_opencv_video=OFF
            -DBUILD_opencv_gapi=OFF
            -DBUILD_opencv_ts=OFF
            -DBUILD_opencv_java=OFF
            -DBUILD_opencv_apps=OFF
            -DOPENCV_GENERATE_PKGCONFIG=ON
            RESULT_VARIABLE CONFIG_RESULT
            OUTPUT_VARIABLE CONFIG_OUTPUT
            ERROR_VARIABLE CONFIG_ERROR
        )
        if(CONFIG_RESULT EQUAL 0)
            message(STATUS "OpenCV configuration successful")
        else()
            message(STATUS "OpenCV configuration output: ${CONFIG_OUTPUT}")
            message(STATUS "OpenCV configuration error: ${CONFIG_ERROR}")
            message(FATAL_ERROR "OpenCV configuration failed with result: ${CONFIG_RESULT}")
        endif()

        # Build libjpeg-turbo explicitly
        message(STATUS "Building libjpeg-turbo explicitly...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build ${OPENCV_BUILD_DIR} --target libjpeg-turbo
            RESULT_VARIABLE JPEG_BUILD_RESULT
            OUTPUT_VARIABLE JPEG_BUILD_OUTPUT
            ERROR_VARIABLE JPEG_BUILD_ERROR
        )
        if(JPEG_BUILD_RESULT EQUAL 0)
            message(STATUS "libjpeg-turbo build successful")
        else()
            message(STATUS "libjpeg-turbo build output: ${JPEG_BUILD_OUTPUT}")
            message(STATUS "libjpeg-turbo build error: ${JPEG_BUILD_ERROR}")
            message(FATAL_ERROR "libjpeg-turbo build failed with result: ${JPEG_BUILD_RESULT}")
        endif()

        # Build OpenCV
        message(STATUS "Building OpenCV... This will take some time...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build ${OPENCV_BUILD_DIR}
            RESULT_VARIABLE BUILD_RESULT
            OUTPUT_VARIABLE BUILD_OUTPUT
            ERROR_VARIABLE BUILD_ERROR
        )
        if(BUILD_RESULT EQUAL 0)
            message(STATUS "OpenCV build successful")
        else()
            message(STATUS "OpenCV build output: ${BUILD_OUTPUT}")
            message(STATUS "OpenCV build error: ${BUILD_ERROR}")
            message(FATAL_ERROR "OpenCV build failed with result: ${BUILD_RESULT}")
        endif()

        # Install OpenCV
        message(STATUS "Installing OpenCV...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --install ${OPENCV_BUILD_DIR}
            RESULT_VARIABLE INSTALL_RESULT
            OUTPUT_VARIABLE INSTALL_OUTPUT
            ERROR_VARIABLE INSTALL_ERROR
        )
        if(INSTALL_RESULT EQUAL 0)
            message(STATUS "OpenCV installation successful")
        else()
            message(STATUS "OpenCV install output: ${INSTALL_OUTPUT}")
            message(STATUS "OpenCV install error: ${INSTALL_ERROR}")
            message(FATAL_ERROR "OpenCV installation failed with result: ${INSTALL_RESULT}")
        endif() 
    endif()
    message(STATUS "OpenCV fetched and installed.")
else()
    message(STATUS "OpenCV already exists at ${OPENCV_INSTALL_DIR}, verifying installation...")
endif()

# Set OpenCV paths explicitly
set(OpenCV_DIR ${OPENCV_INSTALL_DIR}/lib/cmake/opencv4)
set(OpenCV_INCLUDE_DIRS ${OPENCV_INSTALL_DIR}/include/opencv4)

# Find OpenCV
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_DIR}")
    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
    # Explicitly set include directories to avoid source dir references
    set_target_properties(opencv_core opencv_imgproc opencv_highgui opencv_calib3d opencv_photo opencv_imgcodecs
        PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OPENCV_INSTALL_DIR}/include/opencv4"
    )
else()
    message(FATAL_ERROR "OpenCV not found at ${OpenCV_DIR}!")
endif()

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
            -DCMAKE_CXX_FLAGS="-D_XOPEN_SOURCE=700 -D_GNU_SOURCE"
            -DCMAKE_VERBOSE_MAKEFILE=ON  # Enable verbose output
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

    # Clean up
    #message(STATUS "Cleaning up BOOST build artifacts in ${CMAKE_BINARY_DIR}/_deps/...")
    #execute_process(
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/boost-src
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/boost-build
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/boost-subbuild
    #    RESULT_VARIABLE CLEAN_RESULT
    #)
    #if(NOT CLEAN_RESULT EQUAL 0)
    #    message(WARNING "Failed to clean up BOOST build artifacts: ${CLEAN_RESULT}")
    #else()
    #    message(STATUS "BOOST build artifacts cleaned up.")
    #endif()
    message(STATUS "BOOST fetched and installed.")

endif()
# Ensure CMake uses the freshly built Boost
message(STATUS "Verifying BOOST installation...")
set(BOOST_ROOT ${BOOST_INSTALL_DIR})
set(BOOST_DIR ${BOOST_INSTALL_DIR}/lib/cmake/Boost-1.87.0)
set(BOOST_LIBRARY_DIRS ${BOOST_INSTALL_DIR}/lib)
set(BOOST_INCLUDE_DIRS ${BOOST_INSTALL_DIR}/include)
message(STATUS "BOOST can now be found in: ${BOOST_INSTALL_DIR}")

#find_package(Boost REQUIRED COMPONENTS filesystem system iostreams PATHS ${BOOST_INSTALL_DIR}/lib/cmake NO_DEFAULT_PATH)
#if(Boost_FOUND)
#    message(STATUS "Boost found: ${Boost_DIR}")
#    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
#    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
#else()
#    message(FATAL_ERROR "Boost not found at ${BOOST_INSTALL_DIR}/lib/cmake!")
#endif()

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


    # Debug LZ4 setup
    message(STATUS "Debugging LZ4 setup...")
    message(STATUS "LZ4_INSTALL_DIR: ${LZ4_INSTALL_DIR}")
    if(EXISTS "${LZ4_INSTALL_DIR}/lib/pkgconfig/liblz4.pc")
        message(STATUS "Found liblz4.pc at ${LZ4_INSTALL_DIR}/lib/pkgconfig/liblz4.pc")
        file(READ "${LZ4_INSTALL_DIR}/lib/pkgconfig/liblz4.pc" LZ4_PC_CONTENT)
        message(STATUS "liblz4.pc contents:\n${LZ4_PC_CONTENT}")
    else()
        message(WARNING "liblz4.pc not found at ${LZ4_INSTALL_DIR}/lib/pkgconfig/liblz4.pc")
    endif()

    # Set PKG_CONFIG_PATH to help find LZ4 for FLANN
    set(ENV{PKG_CONFIG_PATH} "${LZ4_INSTALL_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
    message(STATUS "PKG_CONFIG_PATH set to: $ENV{PKG_CONFIG_PATH}")


    # Configure FLANN
    message(STATUS "Configuring FLANN...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/flann-src
        -B ${FLANN_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${FLANN_INSTALL_DIR}
        -DZLIB_ROOT=${ZLIB_INSTALL_DIR}
        -DLZ4_ROOT=${LZ4_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=None
        -DWITH_LZ4=ON
        -DBUILD_STATIC_LIBS=ON
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_TESTS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_MATLAB_BINDINGS=ON  # Enable MATLAB bindings
        -DPKG_CONFIG_PATH=${LZ4_INSTALL_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}
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

    # Clean up
    #message(STATUS "Cleaning up FLANN build artifacts in ${CMAKE_BINARY_DIR}/_deps/...")
    #execute_process(
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/flann-src
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/flann-build
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/flann-subbuild
    #    RESULT_VARIABLE CLEAN_RESULT
    #)
    #if(NOT CLEAN_RESULT EQUAL 0)
    #    message(WARNING "Failed to clean up FLANN build artifacts: ${CLEAN_RESULT}")
    #else()
    #    message(STATUS "FLANN build artifacts cleaned up.")
    #endif()
    message(STATUS "FLANN fetched and installed.")
endif()

# Set FLANN paths for later use
message(STATUS "Verifying FLANN installation...")
set(FLANN_ROOT ${FLANN_INSTALL_DIR})
#set(ENV{PKG_CONFIG_PATH} "${FLANN_INSTALL_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")   # Let's try to NOT add this.
message(STATUS "FLANN found, verification complete!")
message(STATUS "FLANN can now be found in: ${FLANN_INSTALL_DIR}")




# ================ PCL ================
set(PCL_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/pcl_src)
set(PCL_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-build)
message(STATUS "PCL_INSTALL_DIR set to: ${PCL_INSTALL_DIR}")
message(STATUS "PCL_BUILD_DIR set to: ${PCL_BUILD_DIR}")

## START OF EXTRA CHECK that LZ4 can be found because of PCL's problematic dependency on LZ4...
#set(LZ4_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/3party/lz4_src/lib/liblz4.a")
#message(STATUS "Checking for LZ4 library at: ${LZ4_LIBRARY_PATH}")
#if(NOT EXISTS ${LZ4_LIBRARY_PATH})
#    message(FATAL_ERROR "LZ4 library not found at ${LZ4_LIBRARY_PATH}")
#endif()

set(Eigen3_DIR "${CMAKE_SOURCE_DIR}/3party/eigen_src/share/eigen3/cmake")
message(STATUS "EIGEN3_DIR set to: ${Eigen3_DIR}")

## END OF EXTRA CHECK
message(STATUS "Checking if PCL_INSTALL_DIR exists: ${PCL_INSTALL_DIR}")
if(NOT EXISTS ${PCL_INSTALL_DIR})
    message(STATUS "Fetching PCL...")
    FetchContent_Declare(
        pcl
        GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl.git
        GIT_TAG master
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/pcl-src
    )
    FetchContent_GetProperties(pcl)
    message(STATUS "PCL populated: ${pcl_POPULATED}")
    if(NOT pcl_POPULATED)
        message(STATUS "Populating PCL...")
        FetchContent_Populate(pcl)
        message(STATUS "Configuring PCL build...")
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/pcl-src
            -B ${PCL_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${PCL_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DPCL_SHARED_LIBS=OFF
            -DLZ4_ROOT=${LZ4_INSTALL_DIR}                       # Point to previously installed LZ4 dir
            "-DCMAKE_SHARED_LINKER_FLAGS=${LZ4_LIBRARY_PATH}"   # Full path here
            -DZLIB_ROOT=${ZLIB_INSTALL_DIR}                     # Point to previously installed ZLIB dir
            -DEigen3_DIR=${EIGEN_INSTALL_DIR} #/share/eigen3/cmake # Point to previously installed eigen dir # CAN BE REMOVED?
            -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/3party/eigen_src;${CMAKE_PREFIX_PATH}
            -DBOOST_ROOT=${BOOST_INSTALL_DIR}                   # Point to previously installed BOOST dir
            -DBoost_DIR=${BOOST_INSTALL_DIR}/lib/cmake/Boost-1.87.0
            -DBOOST_LIBRARY_DIRS=${BOOST_INSTALL_DIR}/lib
            -DFLANN_ROOT=${FLANN_INSTALL_DIR}                   # Point to previously installed flann dir
            -DBUILD_filters=ON
            -DBUILD_keypoints=ON
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

        # Build PCL
        message(STATUS "Building PCL...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build ${PCL_BUILD_DIR}
            RESULT_VARIABLE PCL_BUILD_RESULT
        )
        if(NOT PCL_BUILD_RESULT EQUAL 0)
            # Capture build output for debugging if it fails
            execute_process(
                COMMAND ${CMAKE_COMMAND} --build ${PCL_BUILD_DIR}
                OUTPUT_VARIABLE PCL_BUILD_OUTPUT
                ERROR_VARIABLE PCL_BUILD_ERROR
            )
            message(STATUS "PCL build output: ${PCL_BUILD_OUTPUT}")
            message(STATUS "PCL build error output: ${PCL_BUILD_ERROR}")
            message(FATAL_ERROR "PCL build failed with result: ${PCL_BUILD_RESULT}")
        endif()

        # Install PCL
        message(STATUS "Installing PCL...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --install ${PCL_BUILD_DIR}
            RESULT_VARIABLE PCL_INSTALL_RESULT
        )
        if(NOT PCL_INSTALL_RESULT EQUAL 0)
            # Capture install output for debugging if it fails
            execute_process(
                COMMAND ${CMAKE_COMMAND} --install ${PCL_BUILD_DIR}
                OUTPUT_VARIABLE PCL_INSTALL_OUTPUT
                ERROR_VARIABLE PCL_INSTALL_ERROR
            )
            message(STATUS "PCL install output: ${PCL_INSTALL_OUTPUT}")
            message(STATUS "PCL install error output: ${PCL_INSTALL_ERROR}")
            message(FATAL_ERROR "PCL installation failed with result: ${PCL_INSTALL_RESULT}")
        endif()

    endif()
endif()
message(STATUS "Adding PCL_INSTALL_DIR to CMAKE_PREFIX_PATH: ${PCL_INSTALL_DIR}")
list(APPEND CMAKE_PREFIX_PATH ${PCL_INSTALL_DIR})
set(PCL_ROOT ${PCL_INSTALL_DIR})
message(STATUS "PCL can now be found in: ${PCL_INSTALL_DIR}")

# ================ PCL Include Fix ================
find_package(PCL REQUIRED PATHS ${PCL_INSTALL_DIR}/share/pcl-1.15 NO_DEFAULT_PATH)
if(PCL_FOUND)
    message(STATUS "PCL found: ${PCL_DIR}")
    message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
    message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
else()
    message(FATAL_ERROR "PCL not found at ${PCL_INSTALL_DIR}/share/pcl-1.15!")
endif()


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
        FetchContent_Populate(cnpy) # Only populate, don’t configure yet
        message(STATUS "CNPY populated to: ${CMAKE_BINARY_DIR}/_deps/cnpy-src")

        # Patch CNPY's CMakeLists.txt
        message(STATUS "Patching CNPY CMakeLists.txt...")
        set(CNPY_CMAKELISTS "${CMAKE_BINARY_DIR}/_deps/cnpy-src/CMakeLists.txt")
        file(READ ${CNPY_CMAKELISTS} CNPY_CMAKELISTS_CONTENT)
        string(REPLACE
            "CMAKE_MINIMUM_REQUIRED(VERSION 3.0 FATAL_ERROR)"
            "CMAKE_MINIMUM_REQUIRED(VERSION 3.5)" # Match your project’s version
            CNPY_CMAKELISTS_CONTENT_MODIFIED
            ${CNPY_CMAKELISTS_CONTENT}
        )
        file(WRITE ${CNPY_CMAKELISTS} "${CNPY_CMAKELISTS_CONTENT_MODIFIED}")
        message(STATUS "CNPY CMakeLists.txt patched to version 3.5")

        # Now configure CNPY
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
            OUTPUT_VARIABLE CONFIG_OUTPUT
            ERROR_VARIABLE CONFIG_ERROR
        )
        if(CONFIG_RESULT EQUAL 0)
            message(STATUS "CNPY configuration successful")
        else()
            message(STATUS "CNPY configuration output: ${CONFIG_OUTPUT}")
            message(STATUS "CNPY configuration error: ${CONFIG_ERROR}")
            message(FATAL_ERROR "CNPY configuration failed with result: ${CONFIG_RESULT}")
        endif()

        # Build CNPY
        message(STATUS "Building CNPY in: ${CNPY_BUILD_DIR}")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build ${CNPY_BUILD_DIR}
            RESULT_VARIABLE BUILD_RESULT
            OUTPUT_VARIABLE BUILD_OUTPUT
            ERROR_VARIABLE BUILD_ERROR
        )
        if(BUILD_RESULT EQUAL 0)
            message(STATUS "CNPY build successful")
        else()
            message(STATUS "CNPY build output: ${BUILD_OUTPUT}")
            message(STATUS "CNPY build error: ${BUILD_ERROR}")
            message(FATAL_ERROR "CNPY build failed with result: ${BUILD_RESULT}")
        endif()

        # Install CNPY
        message(STATUS "Installing CNPY to: ${CNPY_INSTALL_DIR}")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --install ${CNPY_BUILD_DIR}
            RESULT_VARIABLE INSTALL_RESULT
            OUTPUT_VARIABLE INSTALL_OUTPUT
            ERROR_VARIABLE INSTALL_ERROR
        )
        if(INSTALL_RESULT EQUAL 0)
            message(STATUS "CNPY installation successful")
        else()
            message(STATUS "CNPY install output: ${INSTALL_OUTPUT}")
            message(STATUS "CNPY install error: ${INSTALL_ERROR}")
            message(FATAL_ERROR "CNPY installation failed with result: ${INSTALL_RESULT}")
        endif()

        # Clean up
        #message(STATUS "Cleaning up CNPY build artifacts in ${CMAKE_BINARY_DIR}/_deps/...")
        #execute_process(
        #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/cnpy-src
        #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/cnpy-build
        #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/cnpy-subbuild
        #    RESULT_VARIABLE CLEAN_RESULT
        #)
        #if(NOT CLEAN_RESULT EQUAL 0)
        #    message(WARNING "Failed to clean up CNPY build artifacts: ${CLEAN_RESULT}")
        #else()
        #    message(STATUS "CNPY build artifacts cleaned up.")
        #endif()

        message(STATUS "CNPY processing complete")
    else()
        message(STATUS "CNPY content already populated at: ${CMAKE_BINARY_DIR}/_deps/cnpy-src")
    endif()
else()
    message(STATUS "CNPY already exists at ${CNPY_INSTALL_DIR}, skipping fetch and build")
endif()

# ================ Sophus ================
set(SOPHUS_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3party/sophus_src)
set(SOPHUS_BUILD_DIR ${CMAKE_BINARY_DIR}/_deps/sophus-build)
message(STATUS "Checking for Sophus at: ${SOPHUS_INSTALL_DIR}")

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
        message(STATUS "Sophus content not populated, starting population process")
        FetchContent_Populate(sophus)
        message(STATUS "Sophus populated to: ${CMAKE_BINARY_DIR}/_deps/sophus-src")
    endif()

    # Configure Sophus
    message(STATUS "Configuring Sophus...")
    execute_process(
        COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_BINARY_DIR}/_deps/sophus-src
        -B ${SOPHUS_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${SOPHUS_INSTALL_DIR}
        -DEIGEN_DIR=${EIGEN_INSTALL_DIR}
        -DBUILD_SOPHUS_TESTS=OFF
        -DBUILD_SOPHUS_EXAMPLES=OFF
        RESULT_VARIABLE CONFIG_RESULT
        OUTPUT_VARIABLE CONFIG_OUTPUT
        ERROR_VARIABLE CONFIG_ERROR
    )
    if(CONFIG_RESULT EQUAL 0)
        message(STATUS "Sophus configuration successful")
    else()
        message(STATUS "Sophus configuration output: ${CONFIG_OUTPUT}")
        message(STATUS "Sophus configuration error: ${CONFIG_ERROR}")
        message(FATAL_ERROR "Sophus configuration failed with result: ${CONFIG_RESULT}")
    endif()

    # Build Sophus
    message(STATUS "Building Sophus...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build ${SOPHUS_BUILD_DIR}
        RESULT_VARIABLE BUILD_RESULT
        OUTPUT_VARIABLE BUILD_OUTPUT
        ERROR_VARIABLE BUILD_ERROR
    )
    if(BUILD_RESULT EQUAL 0)
        message(STATUS "Sophus build successful")
    else()
        message(STATUS "Sophus build output: ${BUILD_OUTPUT}")
        message(STATUS "Sophus build error: ${BUILD_ERROR}")
        message(FATAL_ERROR "Sophus build failed with result: ${BUILD_RESULT}")
    endif()

    # Install Sophus
    message(STATUS "Installing Sophus...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} --install ${SOPHUS_BUILD_DIR}
        RESULT_VARIABLE INSTALL_RESULT
        OUTPUT_VARIABLE INSTALL_OUTPUT
        ERROR_VARIABLE INSTALL_ERROR
    )
    if(INSTALL_RESULT EQUAL 0)
        message(STATUS "Sophus installation successful")
    else()
        message(STATUS "Sophus install output: ${INSTALL_OUTPUT}")
        message(STATUS "Sophus install error: ${INSTALL_ERROR}")
        message(FATAL_ERROR "Sophus installation failed with result: ${INSTALL_RESULT}")
    endif()

    # Clean up
    #message(STATUS "Cleaning up Sophus build artifacts...")
    #execute_process(
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/sophus-src
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/sophus-build
    #    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/_deps/sophus-subbuild
    #    RESULT_VARIABLE CLEAN_RESULT
    #)
    #if(NOT CLEAN_RESULT EQUAL 0)
    #    message(WARNING "Failed to clean up Sophus build artifacts: ${CLEAN_RESULT}")
    #else()
    #    message(STATUS "Sophus build artifacts cleaned up.")
    #endif()
else()
    message(STATUS "Sophus already exists at ${SOPHUS_INSTALL_DIR}, skipping fetch and build")
endif()
message(STATUS "Sophus can now be found in: ${SOPHUS_INSTALL_DIR}")



#message(STATUS "PROCEEDING WITH OPENCV FIX...")
# !!!!!!!!!!!!!!!!! Verify OpenCV !!!!!!!!!!!!!!!!
#find_package(OpenCV REQUIRED PATHS ${OPENCV_INSTALL_DIR}/lib/cmake/opencv4 NO_DEFAULT_PATH)
#if(OpenCV_FOUND)
#    message(STATUS "OpenCV found: ${OpenCV_DIR}")
#    message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
#    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
#    include_directories(${OpenCV_INCLUDE_DIRS})
#else()
#    message(FATAL_ERROR "OpenCV not found at ${OPENCV_INSTALL_DIR}/lib/cmake/opencv4!")
#endif()






# Find OpenMP (required by PCL)
# ================ OPENMP Include Fix ================
#find_package(OpenMP REQUIRED)
#if(OpenMP_FOUND)
#    message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
#else()
#    message(FATAL_ERROR "OpenMP not found!")
#endif()

# ================ Subdirectories ================
# add_subdirectory(minkindr)
# add_subdirectory(tf2)
# add_subdirectory(cartesian3dgrid)
add_subdirectory(external)
add_subdirectory(app)