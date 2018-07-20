cmake_minimum_required (VERSION 2.6)
project (DepthModel)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include)
find_package(OpenCV REQUIRED)

add_library(DepthModel
    ./src/RayLine.cpp
)

target_link_libraries(DepthModel ${OpenCV_LIBS})
