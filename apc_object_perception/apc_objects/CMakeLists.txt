
cmake_minimum_required(VERSION 2.8)
project(apc_objects)

find_package(catkin REQUIRED COMPONENTS roslib cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES apc_objects
)


include_directories(${catkin_INCLUDE_DIRS} include ${Eigen_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_library(apc_objects
	src/apc_objects.cpp
)
target_link_libraries(apc_objects
	${catkin_LIBRARIES}
)
