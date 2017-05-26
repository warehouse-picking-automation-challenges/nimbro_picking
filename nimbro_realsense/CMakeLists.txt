
cmake_minimum_required(VERSION 2.8)
project(nimbro_realsense)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nodelet
	image_geometry
	image_transport
	pcl_ros
	camera_info_manager
	depth_filler
	config_server
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fopenmp")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fopenmp")

add_library(nimbro_realsense_depth_fusion
	src/depth_fusion.cpp
)
target_link_libraries(nimbro_realsense_depth_fusion
	${catkin_LIBRARIES}
)

add_library(nimbro_realsense_convert_metric
	src/convert_metric.cpp
)
target_link_libraries(nimbro_realsense_convert_metric
	${catkin_LIBRARIES}
)

add_executable(measure_latency
	src/measure_latency.cpp
)
target_link_libraries(measure_latency
	${catkin_LIBRARIES}
)

find_library(REALSENSE_LIBRARY realsense)
if(NOT REALSENSE_LIBRARY)
	message(WARNING "librealsense not found, nimbro_realsense driver will not be built.")
	return()
endif()

add_library(nimbro_realsense
	src/realsense.cpp
	src/camera.cpp
)
target_link_libraries(nimbro_realsense
	${catkin_LIBRARIES}
	${REALSENSE_LIBRARY}
)

add_library(nimbro_realsense_stereo
	src/stereo.cpp
	src/camera.cpp
)
target_link_libraries(nimbro_realsense_stereo
	${catkin_LIBRARIES}
	${REALSENSE_LIBRARY}
)
