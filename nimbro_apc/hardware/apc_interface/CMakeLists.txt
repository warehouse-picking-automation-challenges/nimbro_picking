cmake_minimum_required(VERSION 2.8)
project(apc_interface)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	std_srvs
	plot_msgs
	roscpp
	hardware_interface
	controller_manager
	config_server
)

# robotcontrol 0.0.0 contains a dangerous bug (immediate fade-in), so we
# require at least 0.1.0.
#find_package(robotcontrol 0.1.1 REQUIRED)


add_service_files(FILES
	DimLight.srv
	SuctionStrength.srv
)
add_message_files(FILES ControllerState.msg)
generate_messages()
catkin_package(
	CATKIN_DEPENDS message_runtime message_generation
	INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_CURRENT_BINARY_DIR} include)

find_package(libucomm)
if(libucomm_FOUND)
	libucomm_wrap_msg(MSG_SRC protocol/apc_proto.msg)
	add_definitions(-DHAVE_LIBUCOMM=1)
	add_subdirectory(firmware)
	#add_subdirectory(firmware-tests)

	find_package(Boost REQUIRED COMPONENTS program_options)

	#find_library(READLINE_LIBRARY readline)
	#find_path(READLINE_INCLUDE_DIR readline/readline.h)

	#include_directories(${READLINE_INCLUDE_DIR})

	add_executable(controller_tool
		${MSG_SRC}
		src/apc_controller.cpp
		src/controller_tool/controller_tool.cpp
	)
	target_link_libraries(controller_tool
		${Boost_LIBRARIES}
		${catkin_LIBRARIES}
	#        ${READLINE_LIBRARY}
	)

	add_executable(apc_interface
		${MSG_SRC}
		src/apc_controller.cpp
		src/apc_interface.cpp
	)

	add_dependencies(apc_interface
		apc_interface_generate_messages_cpp
	)

	target_link_libraries(apc_interface
		${catkin_LIBRARIES}
	)

else()
	message(WARNING "libucomm not found - apc hw interface will not be built.")
endif()
#add_library(apc_interface
#        ${MSG_SRC}
#)

#add_dependencies(apc_interface
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_link_libraries(apc_interface
#        ${catkin_LIBRARIES}
#        ${robotcontrol_LIBRARIES}
#)

find_package(rqt_gui REQUIRED)
find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

include_directories(${rqt_gui_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

qt4_wrap_cpp(STATUS_DISPLAY_MOC
	src/status_display/status_display.h
)
qt4_wrap_ui(STATUS_DISPLAY_UI
	src/status_display/status_display.ui
)

add_library(apc_interface_status_display
	${STATUS_DISPLAY_MOC}
	${STATUS_DISPLAY_UI}
	src/status_display/status_display.cpp
)
target_link_libraries(apc_interface_status_display
	${catkin_LIBRARIES}
	${rqt_gui_LIBRARIES}
)
add_dependencies(apc_interface_status_display
	${PROJECT_NAME}_generate_messages_cpp
)
