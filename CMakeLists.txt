cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 17)
project(Graph)


# Option
SET(CMAKE_CXX_COMPILER g++-8)
add_definitions( -DGRAPHVIZ )
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG " -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Ofast -Wall")

include_directories(include)
add_executable(test main.cpp)
