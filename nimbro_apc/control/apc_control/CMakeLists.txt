
cmake_minimum_required(VERSION 2.8)
project(apc_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp genmsg geometry_msgs actionlib cmake_modules
	config_server
	actionlib_msgs
	nimbro_fsm
	nimbro_keyframe_server
	eigen_conversions
	tf_conversions
	std_srvs
	tf
	apc_shelf_model
	apc_shelf_registration
	apc_objects
	message_generation
	moveit_ros_planning_interface
	moveit_core
)

find_package(Eigen REQUIRED)

add_message_files(FILES
	WorkingItem.msg
	ShelfState.msg
)

generate_messages()

catkin_package(
	INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
	CATKIN_DEPENDS config_server
)



include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})


add_executable(apc_control
	src/control.cpp
	src/apc_database.cpp
	src/states/state.cpp
	src/states/move_arm.cpp
	src/states/scan_pose.cpp
	src/states/perception.cpp
	src/states/compute_motions.cpp
	src/states/picking/special_pick_motions.cpp
	src/states/stowing/special_stow_motions.cpp
	src/states/check_grasp.cpp
	src/states/shelf_regist.cpp
	src/states/reset.cpp
	src/states/flip_funnel.cpp
	src/states/picking/box_pose.cpp
	src/states/picking/pregrasp.cpp
	src/states/picking/grasp.cpp
	src/states/picking/retract.cpp
	src/states/picking/tote_pose.cpp
	src/states/stowing/home_pose.cpp
	src/states/stowing/stow_item.cpp
	src/states/stowing/stow_pose.cpp
	src/states/stowing/tote_grasp.cpp
	src/states/stowing/tote_pregrasp.cpp
	src/states/stowing/tote_retract.cpp
)
add_dependencies(apc_control
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(apc_control
	${catkin_LIBRARIES}
	yaml-cpp
)

find_package(rqt_gui REQUIRED)
find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

include_directories(${rqt_gui_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

qt4_wrap_cpp(CONTROL_DISPLAY_MOC
	src/control_display/control_display.h
)
qt4_wrap_ui(CONTROL_DISPLAY_UI
	src/control_display/control_display.ui
)

add_library(apc_control_display
	${CONTROL_DISPLAY_MOC}
	${CONTROL_DISPLAY_UI}
	src/control_display/control_display.cpp
)
add_dependencies(apc_control_display
	${PROJECT_NAME}_generate_messages_cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


