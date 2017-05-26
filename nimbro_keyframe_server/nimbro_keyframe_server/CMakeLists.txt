
cmake_minimum_required(VERSION 2.8)
project(nimbro_keyframe_server)

find_package(catkin REQUIRED COMPONENTS
	roscpp genmsg geometry_msgs actionlib cmake_modules
	config_server
	message_generation
	actionlib
	moveit_core
	moveit_ros_planning
	reflexxes
	nimbro_collision_detection
)

find_package(Eigen REQUIRED)

find_package(catch_ros)

add_message_files(FILES
	MotionList.msg
	MotionMsg.msg
	KeyFrameMsg.msg
	JointPosition.msg
	JointGroupMsg.msg
	ExecutionFeedback.msg
)

add_action_files(FILES
	PlayMotion.action
	PlayNamedMotion.action
)

add_service_files(FILES
	GetMotion.srv
	SetMotion.srv
	Play.srv
)

generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES nimbro_keyframe_server trajectory_score
	CATKIN_DEPENDS actionlib reflexxes config_server moveit_core moveit_ros_planning
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${rviz_INCLUDE_DIRS})

add_library(trajectory_score
	src/trajectory_score.cpp
)
add_dependencies(trajectory_score
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(trajectory_score
	${catkin_LIBRARIES}
	${moveit_core_LIBRARIES}
	${tf_LIBRARIES}
	${tf_conversion_LIBRARIES}
	${moveit_ros_planning_LIBRARIES}
)

add_library(nimbro_keyframe_server
	src/keyframe.cpp
	src/motion.cpp
	src/keyframe_server.cpp
	src/moveit_player.cpp
	src/ros_control_player.cpp
)
target_link_libraries(nimbro_keyframe_server
	${catkin_LIBRARIES}
	trajectory_score
	yaml-cpp
)
add_dependencies(nimbro_keyframe_server
	nimbro_keyframe_server_generate_messages_cpp
)

add_executable(ros_control_player
	src/ros_control_player_node.cpp
)
target_link_libraries(ros_control_player
	${catkin_LIBRARIES}
	nimbro_keyframe_server
	trajectory_score
)
add_dependencies(ros_control_player
	nimbro_keyframe_server_generate_messages_cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")


# Tests
if(${catch_ros_FOUND})
	include_directories(${catch_ros_INCLUDE_DIRS})
	catch_add_test(motion_test
		test/motion_test.cpp
		src/keyframe.cpp
		src/motion.cpp
	)
	add_dependencies(motion_test
		nimbro_keyframe_server_generate_messages_cpp
	)

	target_link_libraries(motion_test
		${catkin_LIBRARIES}
		yaml-cpp
	)
endif()
