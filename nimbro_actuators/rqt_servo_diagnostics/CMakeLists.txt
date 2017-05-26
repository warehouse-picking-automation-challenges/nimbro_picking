
cmake_minimum_required(VERSION 2.8)
project(rqt_servo_diagnostics)

find_package(catkin REQUIRED COMPONENTS
	rqt_gui_cpp
	actuator_msgs
)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

include_directories(src ${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})

qt4_wrap_cpp(MOC_SRCS
	src/servo_diagnostics.h
	src/servo_model.h
	src/servo_filter_model.h
)

add_library(rqt_servo_diagnostics
	${MOC_SRCS}
	src/servo_diagnostics.cpp
	src/servo_model.cpp
	src/servo_filter_model.cpp
	src/bar_delegate.cpp
)
target_link_libraries(rqt_servo_diagnostics
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)
