cmake_minimum_required(VERSION 2.8.3)
project(opti)
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	message_generation 
	)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

add_message_files(DIRECTORY msg 
   FILES
   euler_values.msg
  azimut_elevation.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opti
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} )
#include_directories(include ../opti/include ${catkin_INCLUDE_DIRS})

###############################################################################
add_library(opti src/libraries/get.cpp)
target_link_libraries(opti ${catkin_LIBRARIES})
add_dependencies(opti ${catkin_EXPORTED_TARGETS})
###############################################################################
add_executable(get_node src/get_node.cpp)
target_link_libraries(get_node opti ${catkin_LIBRARIES})
add_dependencies(get_node ${catkin_EXPORTED_TARGETS})
