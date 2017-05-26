
cmake_minimum_required(VERSION 2.8)
project(apc_motion)

find_package(catkin REQUIRED COMPONENTS
	robotcontrol
	moveit_core
	moveit_ros_planning_interface
	moveit_ros_planning
	nimbro_keyframe_server
	tf
	actionlib
	std_msgs
	geometry_msgs
)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(apc_motion
	src/apc_motion.cpp
	src/body_part.cpp
)
target_link_libraries(apc_motion
	${catkin_LIBRARIES}
)
