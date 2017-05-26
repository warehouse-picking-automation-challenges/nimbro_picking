
cmake_minimum_required(VERSION 2.8)
project(apc_shelf_registration)

find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	actionlib
	message_generation
	actionlib
	nimbro_keyframe_server
	actionlib_msgs
	tf_conversions
	pcl_conversions
	apc_objects
)

add_action_files(FILES
	ShelfRegistration.action
)

generate_messages(DEPENDENCIES
	std_msgs
	actionlib_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(apc_shelf_registration
	src/shelf_registration.cpp
)
add_dependencies(apc_shelf_registration
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(apc_shelf_registration
	${catkin_LIBRARIES}
)
