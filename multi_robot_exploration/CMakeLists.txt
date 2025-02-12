cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_exploration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  move_base_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    geometry_msgs
    move_base_msgs
    actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(multi_robot_exploration src/multi_robot_exploration_node.cpp)
target_link_libraries(multi_robot_exploration ${catkin_LIBRARIES})
