cmake_minimum_required(VERSION 2.8.3)
project(nimbro_collision_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  config_server
  std_msgs
  message_generation
  sensor_msgs
  moveit_core
  moveit_ros_planning
)

find_package(moveit_core REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)

add_message_files(FILES
	CollisionPair.msg
	CollisionPairs.msg
)

add_service_files(FILES
	CollisionDetection.srv
)

generate_messages(DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES razer_teleop
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} 
)

add_executable(nimbro_collision_detection src/CollisionDetectionNode.cpp)

add_dependencies(nimbro_collision_detection
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries( nimbro_collision_detection
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_robot_interaction_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


