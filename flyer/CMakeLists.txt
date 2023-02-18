cmake_minimum_required(VERSION 2.8.3)
project(flyer)

set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  opti
  nav_msgs
  gazebo_msgs
  tgt_pstion
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flyer
  CATKIN_DEPENDS roscpp rospy gazebo_msgs std_msgs geometry_msgs nav_msgs
  DEPENDS system_lib
)


include_directories(include ../flyer/include ${catkin_INCLUDE_DIRS})
include_directories(include ../tgt_pstion/include ${catkin_INCLUDE_DIRS})

###############################################################################
add_library(flyer src/libraries/flyer.cpp)
target_link_libraries(flyer opti target_position_estimation ${catkin_LIBRARIES})
add_dependencies(flyer ${catkin_EXPORTED_TARGETS})
###############################################################################
add_executable(flyer_node src/flyer_node.cpp)
target_link_libraries(flyer_node flyer ${catkin_LIBRARIES})
add_dependencies(flyer_node ${catkin_EXPORTED_TARGETS})
