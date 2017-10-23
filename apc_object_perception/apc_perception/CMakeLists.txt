cmake_minimum_required(VERSION 2.8.3)
project(apc_perception)


find_package(catkin REQUIRED COMPONENTS
	roslib
	apc_objects
	apc_capture
	eigen_conversions
	tf_conversions
	cv_bridge
	image_transport
	pcl_ros
	eigen_conversions
	actionlib
	actionlib_msgs
	geometry_msgs
	apc_densecap
	apc_6dpose
	camera_calibration_parsers
	apc_shelf_model
	apc_segmentation
	config_server
)


find_package(OpenCV REQUIRED)

add_action_files(FILES
	ApcPerception.action
)

generate_messages(DEPENDENCIES
	std_msgs
	sensor_msgs
	actionlib_msgs
	apc_capture
)

find_package(Boost REQUIRED COMPONENTS
	filesystem
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS pcl_ros apc_capture
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")


include_directories(
	${catkin_INCLUDE_DIRS} include ${OpenCV_INCLUDE_DIRS}
)

add_library(apc_perception
	src/perception_nodelet.cpp
	src/perception_nodelet_grasp.cpp
	src/contrib/connectedcomponents.cpp
	src/contrib/husl/husl.c
	src/color_lut.cpp
)
target_link_libraries(apc_perception
	${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)
add_dependencies(apc_perception
	${PROJECT_NAME}_generate_messages_cpp
)
