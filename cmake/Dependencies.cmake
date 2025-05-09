include(ExternalProject)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# function(fetch_install name repo tag)
# 	set(src    		"${CMAKE_BINARY_DIR}/_deps/${name}-src")
# 	set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/3party/${name}_src")
# 	set(build  		"${CMAKE_BINARY_DIR}/_deps/${name}-build")
# 	if(NOT EXISTS 	"${INSTALL_DIR}")

# 		ExternalProject_Add(
# 			${name} #“Internal” name you’ll use in add_dependencies()
# 			GIT_REPOSITORY 	${repo}
# 			GIT_TAG	 		${tag}
# 			SOURCE_DIR    	${src}
# 			BINARY_DIR    	${build}
# 			INSTALL_DIR   	${INSTALL_DIR}
# 			CMAKE_ARGS
# 				-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
# 				${ARGN}
# 			UPDATE_COMMAND ""
# 		)
# 	endif()
# endfunction()

function(fetch_install name repo tag)
	set(src    		"${CMAKE_BINARY_DIR}/_deps/${name}-src")
	set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/3party/${name}_src")
	set(build  		"${CMAKE_BINARY_DIR}/_deps/${name}-build")
	if(NOT EXISTS 	"${INSTALL_DIR}")
        FetchContent_Declare(
            ${name}
            GIT_REPOSITORY ${repo}
            GIT_TAG ${tag}
        )
        FetchContent_GetProperties(${name})
        if(NOT ${name}_POPULATED)
            FetchContent_Populate(${name})
        endif()
        execute_process(
            COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/${name}-src
            -B ${build}
            -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            ${ARGN}
        )

        message(STATUS "Building ${name}...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build ${build} --parallel 2
        )

        message(STATUS "Installing ${name}...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} --install ${build}
        )

	endif()
endfunction()


# ———————————————————
# Zlib
fetch_install(
	zlib
	https://github.com/madler/zlib.git
	v1.3.1
	-DBUILD_SHARED_LIBS=OFF
	-DCMAKE_POSITION_INDEPENDENT_CODE=ON
)
# ———————————————————
# CNPY
fetch_install(
	cnpy
	https://github.com/rogersce/cnpy.git
	master
	-DBUILD_SHARED_LIBS=OFF
	-DZLIB_ROOT=${CMAKE_SOURCE_DIR}/3party/zlib_src
)
# ———————————————————
# lz4
set(LZ4_INSTALL_DIR "${CMAKE_SOURCE_DIR}/3party/lz4_src")
if(NOT EXISTS ${LZ4_INSTALL_DIR})
    message(STATUS "LZ4 not found at ${LZ4_INSTALL_DIR}")
    message(STATUS "Fetching and building LZ4")
    FetchContent_Declare(
        lz4
        GIT_REPOSITORY https://github.com/lz4/lz4.git
        GIT_TAG v1.9.4
        SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/lz4-src
    )
    FetchContent_MakeAvailable(lz4)

    execute_process(
        COMMAND make
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/lz4-src
        RESULT_VARIABLE BUILD_RESULT
    )
    execute_process(
        COMMAND make install PREFIX=${LZ4_INSTALL_DIR}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/lz4-src
        RESULT_VARIABLE INSTALL_RESULT
    )
endif()

# ———————————————————
# Eigen
fetch_install(
	eigen
	https://gitlab.com/libeigen/eigen.git
	3.4.0
	-DBUILD_TESTING=OFF
)

# ———————————————————
# FLANN
set(ENV{PKG_CONFIG_PATH} "${CMAKE_SOURCE_DIR}/3party/lz4_src/lib/pkgconfig")
fetch_install(
	flann
	https://github.com/flann-lib/flann.git
	1.9.2
	-DBUILD_TESTS=OFF
	-DBUILD_EXAMPLES=OFF
	-DUSE_OPENMP=OFF
	-DBUILD_PYTHON_BINDINGS=OFF
	-DBUILD_DOC=OFF
	-DBUILD_MATLAB_BINDINGS=OFF
)

# ———————————————————
# OpenCV
fetch_install(
	opencv
	https://github.com/opencv/opencv.git
	4.6.0
    -DZLIB_ROOT=${CMAKE_SOURCE_DIR}/3party/zlib_src
    -DBUILD_OPENEXR=ON
    -DBUILD_SHARED_LIBS=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_EXAMPLES=OFF
    -DWITH_IPP=OFF
    -DWITH_QUIRC=OFF
    -DWITH_GAPI=OFF
    -DWITH_ADE=OFF
    -DWITH_TBB=ON
    -DBUILD_TBB=ON
    -DBUILD_LIST=core,imgproc,highgui,calib3d,photo
    -DWITH_PROTOBUF=OFF
    -DBUILD_PROTOBUF=OFF
    -DBUILD_opencv_python2=OFF
    -DBUILD_opencv_python3=OFF
)

# ———————————————————
# Boost
set(BOOST_SRC_DIR   "${CMAKE_BINARY_DIR}/_deps/boost-src")
set(BOOST_BUILD_DIR "${CMAKE_BINARY_DIR}/_deps/boost-build")
set(BOOST_INSTALL "${CMAKE_SOURCE_DIR}/3party/boost_src")
if(NOT EXISTS ${BOOST_INSTALL})
	message(STATUS "BOOST")
  	FetchContent_Declare(
  	  boost
  	  GIT_REPOSITORY https://github.com/boostorg/boost.git
  	  GIT_TAG boost-1.87.0
  	)
  	FetchContent_GetProperties(boost)
  	if(NOT boost_POPULATED)
		FetchContent_Populate(boost)
		message(STATUS "BUILD")
		execute_process(
			COMMAND ${BOOST_SRC_DIR}/bootstrap.sh
				--prefix=${BOOST_INSTALL}
				--with-libraries=filesystem,system,iostreams,serialization
				WORKING_DIRECTORY ${BOOST_SRC_DIR}
		)
		message(STATUS "install")
		execute_process(
			COMMAND ${BOOST_SRC_DIR}/b2 install
				--build-dir=${BOOST_BUILD_DIR}
				--prefix=${BOOST_INSTALL}
				threading=multi
				link=static
				runtime-link=static
				--with-filesystem
				--with-system
				--with-iostreams
				--with-serialization
				WORKING_DIRECTORY ${BOOST_SRC_DIR}
			)
  	endif()
endif()
set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_STATIC_RUNTIME     ON)
# ———————————————————
# Sophus
fetch_install(
	sophus
	https://github.com/strasdat/Sophus.git
	1.24.6
    -DEigen3_DIR=${CMAKE_SOURCE_DIR}/3party/eigen_src/share/eigen3/cmake
    -DBUILD_SOPHUS_TESTS=OFF
)
# ———————————————————
# PCL
fetch_install(
    pcl
    https://github.com/PointCloudLibrary/pcl.git
    pcl-1.15.0
    -DBUILD_SHARED_LIBS=OFF
    -DPCL_SHARED_LIBS=OFF
    -DZLIB_ROOT=${CMAKE_SOURCE_DIR}/3party/zlib_src
    -DCMAKE_CXX_FLAGS=-I${CMAKE_SOURCE_DIR}/3party/lz4_src/include
    -DZLIB_LIBRARY=${CMAKE_SOURCE_DIR}/3party/zlib_src/lib
    -DZLIB_INCLUDE_DIR=${CMAKE_SOURCE_DIR}/3party/zlib_src/include
    -DEigen3_DIR=${CMAKE_SOURCE_DIR}/3party/eigen_src/share/eigen3/cmake
    -DBOOST_ROOT=${CMAKE_SOURCE_DIR}/3party/boost_src
    -DBoost_DIR=${CMAKE_SOURCE_DIR}/3party/boost_src/lib/cmake/Boost-1.87.0
    -DFLANN_ROOT=${CMAKE_SOURCE_DIR}/3party/flann_src
    -DCMAKE_CXX_FLAGS="-mavx"
    -DBoost_USE_STATIC_LIBS=ON
    -DBoost_USE_STATIC_RUNTIME=ON
    -DBUILD_visualization=OFF
    -DBUILD_global_tests=OFF
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

list(APPEND CMAKE_PREFIX_PATH
  "${CMAKE_SOURCE_DIR}/3party/zlib_src"
  "${CMAKE_SOURCE_DIR}/3party/cnpy_src"
  "${CMAKE_SOURCE_DIR}/3party/eigen_src"
  "${CMAKE_SOURCE_DIR}/3party/flann_src"
  "${CMAKE_SOURCE_DIR}/3party/opencv_src"
  "${CMAKE_SOURCE_DIR}/3party/boost_src"
  "${CMAKE_SOURCE_DIR}/3party/pcl_src"
)

find_package(ZLIB   REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(FLANN  REQUIRED)
find_package(Boost  REQUIRED)
find_package(PCL    REQUIRED)

add_library(deps INTERFACE)

target_include_directories(deps INTERFACE
  ${CMAKE_SOURCE_DIR}/3party/zlib_src
  ${CMAKE_SOURCE_DIR}/3party/cnpy_src
  ${CMAKE_SOURCE_DIR}/3party/eigen_src
  ${CMAKE_SOURCE_DIR}/3party/opencv_src
  ${CMAKE_SOURCE_DIR}/3party/sophus_src
  ${CMAKE_SOURCE_DIR}/3party/pcl_src
  ${CMAKE_SOURCE_DIR}/3party/lz4_src

  # plus any CMake‐found pkg’s include vars:
  ${CMAKE_SOURCE_DIR}/3party/zlib_src/include
  ${CMAKE_SOURCE_DIR}/3party/eigen_src/include
  ${CMAKE_SOURCE_DIR}/3party/opencv_src/include/opencv4
  ${CMAKE_SOURCE_DIR}/3party/pcl_src/include/pcl-1.15
  ${CMAKE_SOURCE_DIR}/3party/sophus_src/include
  ${CMAKE_SOURCE_DIR}/3party/lz4_src/include
)

target_link_libraries(deps INTERFACE
#   ZLIB::ZLIB
#   Eigen3::Eigen
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/3party/lz4_src/lib/liblz4.a
)