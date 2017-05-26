
cmake_minimum_required(VERSION 2.8)
project(drc_interface)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	actuator_msgs
	catch_ros
)

# robotcontrol 0.0.0 contains a dangerous bug (immediate fade-in), so we
# require at least 0.1.0.
find_package(robotcontrol 0.1.1 REQUIRED)

find_package(libucomm)

# otherwise catkin_package() complains...
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

catkin_package(
	CATKIN_DEPENDS message_runtime
	INCLUDE_DIRS
		include
		${LIBUCOMM_INCLUDE_DIRS}
		${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
	LIBRARIES drc_interface
)

include_directories(${catkin_INCLUDE_DIRS} ${robotcontrol_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_CURRENT_BINARY_DIR} include)

if(libucomm_FOUND)
	libucomm_wrap_msg(MSG_SRC protocol/drc_proto.msg)

	add_custom_target(copy_libucomm ALL
		COMMAND cmake -E copy_if_different ${MSG_SRC} ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/drc_proto.h
		DEPENDS ${MSG_SRC}
	)
	add_custom_target(copy_libucomm_constants ALL
		COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/protocol/drc_proto_constants.h ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/drc_proto_constants.h
		DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/protocol/drc_proto_constants.h
	)

	add_definitions(-DHAVE_LIBUCOMM=1)
	add_subdirectory(firmware)
	add_subdirectory(firmware-tests)
	set(CONTROLLER_LINK_SOURCES src/controller_link.cpp)

	find_package(Boost REQUIRED COMPONENTS program_options)

	find_library(READLINE_LIBRARY readline)
	find_path(READLINE_INCLUDE_DIR readline/readline.h)

	include_directories(${READLINE_INCLUDE_DIR})

	add_executable(controller_tool
		${MSG_SRC}
		src/controller_tool/controller_tool.cpp
		src/controller_tool/controller_tool_dxl.cpp
		src/controller_tool/controller_tool_hand.cpp
	)
	target_link_libraries(controller_tool
		${Boost_LIBRARIES}
		${READLINE_LIBRARY}
	)
else()
	message(WARNING "libucomm not found - momaro hw interface will not be built.")
endif()

add_library(drc_interface
	${MSG_SRC}
	${CONTROLLER_LINK_SOURCES}
	src/joint.cpp
)
target_link_libraries(drc_interface
	${catkin_LIBRARIES}
	${robotcontrol_LIBRARIES}
)
