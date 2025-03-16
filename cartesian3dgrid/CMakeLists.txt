cmake_minimum_required(VERSION 3.10)
project(cartesian3dgrid)

include(FetchContent)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp")

# Define source and header files BEFORE adding the library
file(GLOB SOURCE_FILES src/cartesian3dgrid.cpp src/cartesian3dgrid_IO.cpp)
file(GLOB HEADER_FILES include/*.h)

# Define the 3rd party directory
set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/3party")
set(CNPY_DIR "${THIRD_PARTY_DIR}/cnpy_src")
set(CNPY_BUILD_DIR "${THIRD_PARTY_DIR}/cnpy_build")
set(ZLIB_DIR "${THIRD_PARTY_DIR}/zlib_src")
set(ZLIB_BUILD_DIR "${THIRD_PARTY_DIR}/zlib_build")

# --- Fetch ZLIB ---
if(NOT EXISTS "${ZLIB_DIR}")
    message(STATUS "ZLIB not found, fetching from GitHub...")
    FetchContent_Declare(
        zlib
        GIT_REPOSITORY https://github.com/madler/zlib.git
        GIT_TAG master
    )

    file(MAKE_DIRECTORY ${ZLIB_DIR})

    FetchContent_MakeAvailable(zlib)

    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/zlib-src
            -B ${ZLIB_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${ZLIB_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    )

    execute_process(COMMAND ${CMAKE_COMMAND} --build ${ZLIB_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${ZLIB_BUILD_DIR})
    file(REMOVE_RECURSE ${ZLIB_BUILD_DIR})
endif()

# Pass these paths to CMake so it can find Zlib
set(CMAKE_PREFIX_PATH "${ZLIB_DIR}" ${CMAKE_PREFIX_PATH})
set(CMAKE_LIBRARY_PATH "${ZLIB_DIR}/lib" ${CMAKE_LIBRARY_PATH})
set(CMAKE_INCLUDE_PATH "${ZLIB_DIR}/include" ${CMAKE_INCLUDE_PATH})

# --- Fetch CNPY ---
if(NOT EXISTS "${CNPY_DIR}")
    message(STATUS "CNPY not found in ${THIRD_PARTY_DIR}, fetching from GitHub...")

    FetchContent_Declare(
        cnpy
        GIT_REPOSITORY https://github.com/rogersce/cnpy.git
        GIT_TAG master
    )
    file(MAKE_DIRECTORY ${CNPY_DIR})

    FetchContent_MakeAvailable(cnpy)
    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -S ${CMAKE_BINARY_DIR}/_deps/cnpy-src
            -B ${CNPY_BUILD_DIR}
            -DCMAKE_INSTALL_PREFIX=${CNPY_DIR}
            -DBUILD_SHARED_LIBS=OFF
            -DZLIB_INCLUDE_DIR=${ZLIB_DIR}/include
            -DZLIB_LIBRARY=${ZLIB_DIR}/lib/libz.a
    )

    execute_process(COMMAND ${CMAKE_COMMAND} --build ${CNPY_BUILD_DIR})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${CNPY_BUILD_DIR})
    file(REMOVE_RECURSE ${CNPY_BUILD_DIR})
endif()

# Set CNPY paths
set(CNPY_INCLUDE_DIR "${CNPY_DIR}/include")
set(CNPY_LIBRARY "${CNPY_DIR}/lib/libcnpy.a")

# Create the main project library
add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

message(STATUS "CNPY Include: ${CNPY_INCLUDE_DIR}")
message(STATUS "CNPY Library: ${CNPY_LIBRARY}")
message(STATUS "ZLIB Include: ${ZLIB_INCLUDE_DIR}")
message(STATUS "ZLIB Library: ${ZLIB_LIBRARY}")

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CNPY_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${ZLIB_DIR}/include
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${CNPY_LIBRARY} ${ZLIB_DIR}/lib/libz.a)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)