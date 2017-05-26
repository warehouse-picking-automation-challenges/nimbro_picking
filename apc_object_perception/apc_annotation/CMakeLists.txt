cmake_minimum_required(VERSION 2.8) 
project(apc_annotation)         

find_package(catkin REQUIRED COMPONENTS
	roslib
	apc_objects
)

find_package(Qt4 REQUIRED 
	COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})        
include_directories(${CMAKE_CURRENT_BINARY_DIR})


find_package(OpenCV REQUIRED)


catkin_package()               
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include_directories(src ${CMAKE_CURRENT_BINARY_DIR})

qt4_wrap_cpp(MOC_SRCS
	src/apc_annotation.h
	src/item_dialog.h
	src/annotation_view.h
)

qt4_wrap_ui(UI_HDRS
	src/apc_annotation.ui
	src/item_dialog.ui
)

add_executable(apc_annotation
	${MOC_SRCS}
	${UI_HDRS}
	src/main.cpp
	src/apc_annotation.cpp
	src/annotation_view.cpp
	src/annotation_model.cpp
	src/item_dialog.cpp
	src/item_model.cpp
)
target_link_libraries(apc_annotation
	${catkin_LIBRARIES} 
	${QT_LIBRARIES}
	${OpenCV_LIBS}
	yaml-cpp
)
