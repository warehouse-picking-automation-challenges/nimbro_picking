
cmake_minimum_required(VERSION 2.8)

project(nimbro_fsm)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	rqt_gui_cpp
)

include_directories(${catkin_INCLUDE_DIRS} include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_message_files(FILES
	Info.msg
	Status.msg
)

add_service_files(FILES
	ChangeState.srv
)

generate_messages(
	DEPENDENCIES std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES nimbro_fsm
)

add_library(nimbro_fsm
	src/fsm.cpp
	src/fsm_ros.cpp
	src/impl/utils.cpp
)
target_link_libraries(nimbro_fsm
	${catkin_LIBRARIES}
)
add_dependencies(nimbro_fsm
	nimbro_fsm_generate_messages_cpp
)

add_executable(nimbro_fsm_test
	src/test.cpp
)
target_link_libraries(nimbro_fsm_test
	${catkin_LIBRARIES}
	nimbro_fsm
)
add_dependencies(nimbro_fsm_test
	nimbro_fsm_generate_messages_cpp
)

add_subdirectory(gui)
