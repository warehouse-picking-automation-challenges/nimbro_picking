cmake_minimum_required(VERSION 2.8.3)
project(apc_world)

find_package(catkin REQUIRED COMPONENTS
  nimbro_keyframe_server
  apc_objects
  moveit_core
  moveit_ros_planning
  roscpp
  tf
  tf_conversions
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apc_world
#  CATKIN_DEPENDS actionlib nimbro_keyframe_server roscpp
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(apc_world
  src/apc_world.cpp
)

target_link_libraries(apc_world
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${tf_LIBRARIES}
  ${tf_conversion_LIBRARIES}
)


add_executable(sim_test
  src/apc_world.cpp
  test/simulation_test.cpp
)


target_link_libraries(sim_test
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
)



