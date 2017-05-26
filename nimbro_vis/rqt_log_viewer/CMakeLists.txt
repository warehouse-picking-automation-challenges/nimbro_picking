
cmake_minimum_required(VERSION 2.8)

project(rqt_log_viewer)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rosgraph_msgs
	rqt_gui_cpp
)

include_directories(${catkin_INCLUDE_DIRS})

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Widgets REQUIRED)
	find_package(Qt5WebKitWidgets REQUIRED)
	find_package(Qt5DBus REQUIRED)
	set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5WebKitWidgets_LIBRARIES} ${Qt5DBus_LIBRARIES})
else()
	find_package(Qt4 COMPONENTS QtCore QtGui QtWebKit QtDBus REQUIRED)
	include(${QT_USE_FILE})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include_directories(${CMAKE_CURRENT_BINARY_DIR} src)

catkin_package()

set(UI_FILES
	src/rqt_log_viewer.ui
)
set(H_FILES
	src/log_model.h
	src/rqt_log_viewer.h
	src/log_view.h
	src/node_model.h
	src/checkboxdelegate.h
	src/log_filter.h
	src/node_backend.h
	src/filter_line_edit.h
	src/message_display.h
	src/combobox_delegate.h
	src/editors/kdevelop.h
)

if(USE_QT5)
	qt5_wrap_ui(UIC_FILES ${UI_FILES})
	qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
	qt4_wrap_ui(UIC_FILES ${UI_FILES})
	qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

add_library(rqt_log_viewer
	${UIC_FILES}
	${MOC_FILES}
	src/rqt_log_viewer.cpp
	src/log_model.cpp
	src/log_view.cpp
	src/node_model.cpp
	src/checkboxdelegate.cpp
	src/log_filter.cpp
	src/node_backend.cpp
	src/filter_line_edit.cpp
	src/message_display.cpp
	src/combobox_delegate.cpp
	src/editors/kdevelop.cpp
)
target_link_libraries(rqt_log_viewer
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

install(FILES plugin_rqt_gui.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
