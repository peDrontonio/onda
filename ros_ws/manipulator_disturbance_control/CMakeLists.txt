cmake_minimum_required(VERSION 3.0.2)
project(manipulator_disturbance_control)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  manipulator_msgs
  tf2_ros
  tf2_geometry_msgs
)

## Catkin package macro
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    gazebo_msgs
    visualization_msgs
    manipulator_msgs
    tf2_ros
    tf2_geometry_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(disturbance_applier_node 
  src/disturbance_applier_node.cpp
)

add_executable(test_disturbance_client 
  src/test_disturbance_client.cpp
)

## Add dependencies
add_dependencies(disturbance_applier_node 
  ${catkin_EXPORTED_TARGETS}
  manipulator_msgs_generate_messages_cpp
)

add_dependencies(test_disturbance_client 
  ${catkin_EXPORTED_TARGETS}
  manipulator_msgs_generate_messages_cpp
)

## Link libraries
target_link_libraries(disturbance_applier_node
  ${catkin_LIBRARIES}
)

target_link_libraries(test_disturbance_client
  ${catkin_LIBRARIES}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/test_disturbance_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
