
cmake_minimum_required(VERSION 2.8)
project(apc_densecap)

find_package(catkin REQUIRED COMPONENTS
	nodelet
	roslib
	cv_bridge
	cmake_modules
	apc_capture
	tf_conversions
	eigen_conversions
)

find_library(LIBLINEAR linear)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES apc_densecap
)

add_definitions(-std=c++11)

find_package(Eigen REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem system program_options)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(proposal_generator
	src/proposal_generator.cpp
)
set_target_properties(proposal_generator PROPERTIES
	COMPILE_FLAGS "-fopenmp"
	LINK_FLAGS "-fopenmp"
)
target_link_libraries(proposal_generator
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
)

set(TORCH_PATH "" CACHE PATH "Path to the torch installation")

if(NOT TORCH_PATH)
	message(STATUS "Please set TORCH_PATH to build apc_densecap.")

	# Build mock apc_densecap
	add_definitions(-DAPC_DENSECAP_MOCK=1)

	add_library(apc_densecap
		src/apc_densecap.cpp
	)
	target_link_libraries(apc_densecap
		${catkin_LIBRARIES}
	)

	return()
endif()

include_directories(${TORCH_PATH}/include)

find_library(LUAJIT_LIB luajit PATHS ${TORCH_PATH}/lib)
find_library(TORCH_LIB TH PATHS ${TORCH_PATH}/lib)
find_library(LUAT_LIB luaT PATHS ${TORCH_PATH}/lib)

if(LIBLINEAR)
	add_library(apc_densecap
		src/sample_db.cpp
		src/rectangle.cpp
		src/svm.cpp
		src/proposal_generator.cpp
		src/apc_densecap.cpp
	)
	set_target_properties(apc_densecap PROPERTIES
		COMPILE_FLAGS "-fopenmp"
		LINK_FLAGS "-fopenmp"
	)
	target_link_libraries(apc_densecap
		yaml-cpp
		${LIBLINEAR}
		${Boost_LIBRARIES}
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		${LUAJIT_LIB}
		${TORCH_LIB}
		${LUAT_LIB}
		${catkin_LIBRARIES}
	)

	add_executable(train
		src/train.cpp
	)
	set_target_properties(train PROPERTIES
		COMPILE_FLAGS "-fopenmp"
		LINK_FLAGS "-fopenmp"
	)
	target_link_libraries(train
		yaml-cpp
		${LIBLINEAR}
		${Boost_LIBRARIES}
		${OpenCV_LIBS}
		apc_densecap
	)
endif()
