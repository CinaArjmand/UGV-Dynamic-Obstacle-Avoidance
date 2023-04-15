cmake_minimum_required(VERSION 2.8.3)
project(a_star_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES a_star_lib
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include
)

add_library(a_star_lib src/AStar.cpp)
add_dependencies(a_star_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(a_star_lib
  ${catkin_LIBRARIES}
)
add_executable(a_star_planner_node src/a_star_planner_node.cpp src/AStar.cpp)
add_dependencies(a_star_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(a_star_planner_node
  ${catkin_LIBRARIES}
  a_star_lib
)


#################################################################################
## Install
#################################################################################
install(TARGETS a_star_planner_node
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
