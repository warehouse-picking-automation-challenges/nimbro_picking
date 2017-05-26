
cmake_minimum_required(VERSION 2.8)
project(nimbro_keyframe_editor)

find_package(catkin REQUIRED COMPONENTS
	rviz
	actionlib
	nimbro_keyframe_server
	plot_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_robot_interaction
	moveit_ros_visualization
# 	momaro_upper_body
)

find_package(rviz REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

message(STATUS "RViz version: ${rviz_VERSION}")
if(${rviz_VERSION} VERSION_LESS 1.12)
	message(STATUS "RViz version is < 1.12 (=> Indigo), using Qt4")

	find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
	include(${QT_USE_FILE})

	qt4_wrap_ui(UI_SRCS
		src/keyframe_editor.ui
	)

	qt4_wrap_cpp(MOC_SRCS
		src/keyframe_editor.h
		src/motion_model.h
		src/ik_view.h
		src/joint_space_view.h
	)
else()
	message(STATUS "RViz version is >= 1.12, using Qt5")

	find_package(Qt5Core)
	find_package(Qt5Widgets)
	set(QT_LIBRARIES Qt5::Core Qt5::Widgets)

	qt5_wrap_ui(UI_SRCS
		src/keyframe_editor.ui
	)

	qt5_wrap_cpp(MOC_SRCS
		src/keyframe_editor.h
		src/motion_model.h
		src/ik_view.h
		src/joint_space_view.h
	)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(nimbro_keyframe_editor
	${UI_SRCS}
	${MOC_SRCS}
	src/keyframe_editor.cpp
	src/motion_model.cpp
	src/ik_view.cpp
	src/joint_space_view.cpp
	src/limit_delegate.cpp
)

set(NIMBRO_KEYFRAME_LIBS
	${catkin_LIBRARIES}
	${moveit_core_LIBRARIES}
	${moveit_ros_planning_LIBRARIES}
	${moveit_ros_robot_interaction_LIBRARIES}
	${RENDER_TOOLS}
	${QT_LIBRARIES}
)

list(REMOVE_DUPLICATES NIMBRO_KEYFRAME_LIBS)

target_link_libraries(nimbro_keyframe_editor
	${NIMBRO_KEYFRAME_LIBS}
	${QT_LIBRARIES}
)
