cmake_minimum_required(VERSION 2.8)
project(apc_capture)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	cv_bridge
	image_transport
	image_geometry
	tf_conversions
	pcl_ros
	eigen_conversions
	camera_calibration_parsers
	rosbag
	geometry_msgs
	apc_objects
	apc_shelf_model
)

find_package(Boost REQUIRED COMPONENTS
		thread
		filesystem
		)

find_package(OpenCV REQUIRED)


include_directories(${catkin_INCLUDE_DIRS} include  ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DAPC_CAPTURE_DEBUG=1")


add_message_files(FILES
		BoxCoordinates.msg
		)
generate_messages(DEPENDENCIES
		std_msgs
		sensor_msgs
		)

catkin_package(
		INCLUDE_DIRS include
		LIBRARIES apc_features apc_tote_features
		CATKIN_DEPENDS pcl_ros
)


add_executable(apc_capture
	src/aic.cpp
)
target_link_libraries(apc_capture
	${catkin_LIBRARIES}
)

add_executable(play_bag
	src/play_bag.cpp
)
target_link_libraries(play_bag
	${catkin_LIBRARIES}
)

#add_executable(apc_boxreg_main
#	src/box_reg_main.cpp
#)
## can remove box_reg_main if needed  ***better to make sure delete this later when this module is stable***
#target_link_libraries(apc_boxreg_main
#	${catkin_LIBRARIES} ${PCL_LIBRARIES}
#)
#add_dependencies(apc_boxreg_main
#	${PROJECT_NAME}_generate_messages_cpp
#)

#add_library(apc_boxreg_nodelet
#	src/box_registration.cpp
#)
#add_dependencies(apc_boxreg_nodelet
#	${PROJECT_NAME}_generate_messages_cpp
#)
#target_link_libraries(apc_boxreg_nodelet
#	${catkin_LIBRARIES} ${PCL_LIBRARIES} apc_features
#)


add_library(apc_features
	src/apc_features.cpp
	src/utils.cpp
	)
target_link_libraries(apc_features
	${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
	)
add_dependencies(apc_features
	${PROJECT_NAME}_generate_messages_cpp
)


add_library(apc_tote_features
		src/apc_tote_features.cpp
)
target_link_libraries(apc_tote_features
		${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)


add_executable(apc_features_exe
	src/features_exe.cpp
)
target_link_libraries(apc_features_exe
	${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} apc_features
)
add_dependencies(apc_features_exe
	${PROJECT_NAME}_generate_messages_cpp
)


add_executable(apc_tote_features_exe
		src/tote_features_exe.cpp
)
target_link_libraries(apc_tote_features_exe
		${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} apc_tote_features
)


#add_executable(tb
#		src/testBed.cpp
#)
#target_link_libraries(tb
#		${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES}
#		)
