cmake_minimum_required(VERSION 2.8.3)
project(apc_ros_player)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nimbro_keyframe_server
  tf_conversions
  tf
  apc_objects
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apc_ros_player
#  CATKIN_DEPENDS ros_control_player
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/apc_ros_player.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
