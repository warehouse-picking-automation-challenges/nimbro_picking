
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

qt4_wrap_ui(UI_HDRS
	statemachine_gui.ui
	jumpstatedialog.ui
)

qt4_wrap_cpp(MOC_SRCS
	statemachine_gui.h
	jumpstatedialog.h
	buttonview.h
)

add_library(nimbro_fsm_gui
	${UI_HDRS}
	${MOC_SRCS}
	statemachine_gui.cpp
	jumpstatedialog.cpp
	buttonview.cpp
)
target_link_libraries(nimbro_fsm_gui
	${catkin_LIBRARIES}
	${rqt_gui_cpp_LIBRARIES}
	${QT_LIBRARIES}
)
add_dependencies(nimbro_fsm_gui
	nimbro_fsm_generate_messages_cpp
)

