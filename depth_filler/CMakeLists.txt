
cmake_minimum_required(VERSION 2.8)
project(depth_filler)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3 -march=native")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(BLAS)

set(ALL_SPARSE_LIBS "")

if(BLAS_FOUND)
	include_directories(${BLAS_INCLUDE_DIR})
	set(ALL_SPARSE_LIBS ${BLAS_LIBRARIES})

	find_package(KLU)
	if(KLU_FOUND)
		include_directories(${KLU_INCLUDES})
		add_definitions(-DKLU_SUPPORT)
		set(ALL_SPARSE_LIBS ${ALL_SPARSE_LIBS} ${KLU_LIBRARIES})
	endif()

	find_package(Umfpack)
	if(UMFPACK_FOUND)
		include_directories(${UMFPACK_INCLUDES})
		add_definitions(-DEIGEN_UMFPACK_SUPPORT)
		set(ALL_SPARSE_LIBS ${ALL_SPARSE_LIBS} ${UMFPACK_LIBRARIES})
	endif()

	if(KLU_FOUND)
		set(SPARSE_LIBS ${BLAS_LIBRARIES} ${KLU_LIBRARIES})
		message(STATUS "Using KLU library as sparse solver")
	elseif(UMFPACK_FOUND)
		set(SPARSE_LIBS ${BLAS_LIBRARIES} ${UMFPACK_LIBRARIES})
		message(STATUS "Using UMFPACK as sparse solver")
	else()
		set(SPARSE_LIBS "")
		message(WARNING "Neither KLU nor UMFPACK found, using slow Eigen solver")
	endif()
else()
	message(WARNING "No BLAS library found, using slow Eigen solver")
endif()

find_package(PCL)
if(PCL_FOUND)
	message(STATUS "PCL support enabled.")
	include_directories(${PCL_INCLUDE_DIRS})
	add_definitions(-DHAVE_PCL)
endif()

find_package(Boost COMPONENTS program_options)

find_package(OpenCV REQUIRED)

set(CUSTOM_EIGEN_INCLUDE_DIR "" CACHE STRING "Include path of custom Eigen 3.3")
if(CUSTOM_EIGEN_INCLUDE_DIR)
	add_definitions(-DCUSTOM_EIGEN="${CUSTOM_EIGEN_INCLUDE_DIR}/Eigen/Core")
endif()

include_directories(include)

################################################################################
# ROS stuff

find_package(catkin QUIET COMPONENTS
	pcl_ros
	nodelet
	image_geometry
	config_server
)

if(catkin_FOUND)
	include_directories(${catkin_INCLUDE_DIRS})

	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES depth_filler
	)
endif()

# end ROS stuff
################################################################################

set_source_files_properties(src/domain_transform.cpp PROPERTIES
	COMPILE_FLAGS -I/home/max/devel/uni/apc/extra/prefix/include/eigen3
)

add_library(depth_filler
	src/depth_filler.cpp
	src/domain_transform.cpp
)
target_link_libraries(depth_filler
	${SPARSE_LIBS}
	${OpenCV_LIBS}
)

if(PCL_FOUND AND Boost_FOUND)
	add_executable(fill_pcd
		src/fill_pcd.cpp
	)
	target_link_libraries(fill_pcd
		depth_filler
		${PCL_LIBRARIES}
		${Boost_LIBRARIES}
	)
endif()


add_executable(solve_test
	src/solve_test/solve_test.cpp
)
target_link_libraries(solve_test
	${BLAS_LIBRARIES}
	${ALL_SPARSE_LIBS}
)

if(catkin_FOUND)
	add_library(depth_filler_nodelet
		src/depth_filler_nodelet.cpp
	)
	target_link_libraries(depth_filler_nodelet
		${catkin_LIBRARIES}
		depth_filler
	)
endif()

################################################################################
# UI

find_package(Qt4)

if(QT4_FOUND AND PCL_FOUND)
	include(${QT_USE_FILE})
	include_directories(${CMAKE_CURRENT_BINARY_DIR})
	include_directories(src/tuner)

	qt4_wrap_ui(TUNER_UI
		src/tuner/fill_tuner.ui
	)

	qt4_wrap_cpp(TUNER_MOC
		src/tuner/fill_tuner.h
	)

	add_executable(fill_tuner
		${TUNER_UI}
		${TUNER_MOC}
		src/tuner/fill_tuner.cpp
		src/tuner/image_widget.cpp
	)
	target_link_libraries(fill_tuner
		depth_filler
		${QT_LIBRARIES}
		${PCL_LIBRARIES}
	)
endif()
