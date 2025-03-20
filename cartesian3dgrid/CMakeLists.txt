cmake_minimum_required(VERSION 3.20)
project(cartesian3dgrid)

set(CMAKE_CXX_STANDARD 14)

# Define source and header files
file(GLOB SOURCE_FILES src/cartesian3dgrid.cpp src/cartesian3dgrid_IO.cpp)
file(GLOB HEADER_FILES include/*.h)

# Define 3rd party directories (set by parent)
set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/3party")
set(CNPY_INSTALL_DIR "${THIRD_PARTY_DIR}/cnpy_src")
set(ZLIB_INSTALL_DIR "${THIRD_PARTY_DIR}/zlib_src")

# Create the library
add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CNPY_INSTALL_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${ZLIB_INSTALL_DIR}/include
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${CNPY_INSTALL_DIR}/lib/libcnpy.a
    ${ZLIB_INSTALL_DIR}/lib/libz.a
    ${OpenCV_LIBS}
)