cmake_minimum_required(VERSION 3.0.2)
project(legged_joy)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  controller_manager_msgs
  ocs2_msgs
  ocs2_core
  ocs2_legged_robot_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    controller_manager_msgs
    ocs2_msgs
    ocs2_core
    ocs2_legged_robot_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xbox_joy_node
  src/xbox_joy.cpp
)

target_link_libraries(xbox_joy_node
  ${catkin_LIBRARIES}
)

# Optional: ensure proper include order
add_dependencies(xbox_joy_node
  ${catkin_EXPORTED_TARGETS}
)
