cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(box)

find_package(PCL 1.8 REQUIRED)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories("/usr/include/eigen3" ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS} "/usr/include/eigen3" ${PCL_INCLUDE_DIRS})


add_executable (box box.cpp)
target_link_libraries (box ${PCL_LIBRARIES})
