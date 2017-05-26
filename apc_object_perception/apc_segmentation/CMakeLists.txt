
cmake_minimum_required(VERSION 2.8)
project(apc_segmentation)

find_package(catkin REQUIRED COMPONENTS
	roslib
	cv_bridge
	cmake_modules
	tf_conversions
	eigen_conversions
	cv_bridge
	apc_densecap
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES apc_segmentation
)

add_definitions(-std=c++11)

find_package(Eigen REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem system program_options)

include_directories(${PCL_INCLUDE_DIRS})

set(TORCH_PATH "" CACHE PATH "Path to the torch installation")

if(NOT TORCH_PATH)
	message(STATUS "Please set TORCH_PATH to build apc_segmentation.")

	# Build mock apc_segmentation
	add_definitions(-DAPC_SEGMENTATION_MOCK=1)

	add_library(apc_segmentation
		src/apc_segmentation.cpp
	)
	target_link_libraries(apc_segmentation
		${catkin_LIBRARIES}
	)

	return()
endif()

include_directories(${TORCH_PATH}/include)

find_library(LUAJIT_LIB luajit PATHS ${TORCH_PATH}/lib)
find_library(TORCH_LIB TH PATHS ${TORCH_PATH}/lib)
find_library(LUAT_LIB luaT PATHS ${TORCH_PATH}/lib)


add_library(apc_segmentation
	src/apc_segmentation.cpp
)
set_target_properties(apc_segmentation PROPERTIES
	COMPILE_FLAGS "-fopenmp"
	LINK_FLAGS "-fopenmp"
)
target_link_libraries(apc_segmentation
	yaml-cpp
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	${LUAJIT_LIB}
	${TORCH_LIB}
	${LUAT_LIB}
	${catkin_LIBRARIES}
)

add_executable(test_segmentation
	src/test_segmentation.cpp
)
target_link_libraries(test_segmentation
	apc_segmentation
)
