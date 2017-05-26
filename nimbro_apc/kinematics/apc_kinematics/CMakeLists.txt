cmake_minimum_required(VERSION 2.8)

project(apc_kinematics)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_ros_planning
	rbdl
	config_server
	robotcontrol
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(apc_kinematics
	src/apc_kinematics.cpp
	src/srdf_cache.cpp
)
target_link_libraries(apc_kinematics
	${catkin_LIBRARIES}
)
