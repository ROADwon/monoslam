cmake_minimum_required(VERSION 3.26)
project(untitled)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

file(GLOB viso
"src/*.h"
"src/*.cpp")
add_executable(untitled main.cpp
        vo_feature.h)
add_executable( untitled ${viso})
target_link_libraries( untitled ${OpenCV_LIBS})