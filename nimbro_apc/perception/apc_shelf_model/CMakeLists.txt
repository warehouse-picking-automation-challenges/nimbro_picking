
cmake_minimum_required(VERSION 2.8)
project(apc_shelf_model)

find_package(catkin REQUIRED COMPONENTS rosconsole)
catkin_package(INCLUDE_DIRS include LIBRARIES apc_shelf_model)


include_directories(include ${catkin_INCLUDE_DIRS})

add_library(apc_shelf_model src/dimensions.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
