cmake_minimum_required (VERSION 2.6)
project (DepthModel)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(./include)
find_package(OpenCV REQUIRED)

# the files are added in the dependences way
add_library(DepthModel
  ./src/Triangle3D.cpp
  ./src/PoseWrapper.cpp
  ./src/CameraRayCalculator.cpp
  ./src/utils.cpp
  ./src/RayLine.cpp
  ./src/ModelContainer.cpp
  ./src/ModelPoint.cpp
  ./src/ModelBuilder.cpp
  ./src/BruteForceModelBuilder.cpp
  ./src/Visualizer.cpp
  ./src/geometry.cpp
  ./src/Box.cpp
  ./src/iterateOctree.cpp
)

target_link_libraries(DepthModel ${OpenCV_LIBS})

add_executable(demo demo.cpp)
target_link_libraries(demo DepthModel ${OpenCV_LIBS})
