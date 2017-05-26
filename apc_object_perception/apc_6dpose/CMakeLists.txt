
cmake_minimum_required(VERSION 2.8)
project(apc_6dpose)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
	LIBRARIES apc_6dpose
)


#set(V4R_DIR /home/aura/software/v4r_final/build/ CACHE PATH "Path to V4R")
#set(EDT_DIR /home/aura/software/v4r_final/3rdparty CACHE PATH "Path to EDT")
find_package(V4R QUIET)

add_definitions(-std=c++11)

find_package(Eigen REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem system program_options)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EDT_INCLUDE_DIR} )

include_directories(/home/lmunoza/software/v4r_final/3rdparty)
include_directories(${V4R_INCLUDE_DIRS})

if(NOT V4R_FOUND)
    message(STATUS "Please set V4R_DIR to build apc_6dpose.")

    # Build mock apc_6dpose
    add_definitions(-DAPC_6DPOSE_MOCK=1)

    add_library(apc_6dpose
            src/apc_6dpose.cpp
    )
    target_link_libraries(apc_6dpose
            ${catkin_LIBRARIES}
    )

    return()
endif()
#find_package(EDT REQUIRED)


if(EDT_FOUND)
    message(STATUS "EDT detected: " ${EDT_INCLUDE_DIRS})
endif(EDT_FOUND)



add_library(apc_6dpose
        src/apc_6dpose.cpp
)
set_target_properties(apc_6dpose PROPERTIES
        COMPILE_FLAGS "-fopenmp"
        LINK_FLAGS "-fopenmp"
)

target_link_libraries(apc_6dpose
        yaml-cpp
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${V4R_LIBS}
)

add_executable(recognizer
        src/recognizer.cpp
)
set_target_properties(recognizer PROPERTIES
        COMPILE_FLAGS "-fopenmp"
        LINK_FLAGS "-fopenmp"
)
target_link_libraries(recognizer
        yaml-cpp
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${V4R_LIBS}
)

add_executable(test_registration
	src/test_registration.cpp
)
target_link_libraries(test_registration
	apc_6dpose
)

add_executable(grasp_annotation
        src/grasp_annotation.cpp
)
target_link_libraries(grasp_annotation
        apc_6dpose ${PCL_LIBRARIES}
)


add_executable(train_linemod_apc_data
        src/train_linemod_apc_data.cpp
)
target_link_libraries(train_linemod_apc_data
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}

)

add_executable(detection_linemod_apc_data
        src/detection_linemod_apc_data.cpp
)
target_link_libraries(detection_linemod_apc_data
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}

)


