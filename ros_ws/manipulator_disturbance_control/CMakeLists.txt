cmake_minimum_required(VERSION 3.8)
project(manipulator_disturbance_control)

## Compile as C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(manipulator_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## Declare executables
add_executable(disturbance_applier_node 
  src/disturbance_applier_node.cpp
)

add_executable(test_disturbance_client 
  src/test_disturbance_client.cpp
)

## Specify dependencies
ament_target_dependencies(disturbance_applier_node
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  manipulator_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(test_disturbance_client
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  manipulator_msgs
  tf2_ros
  tf2_geometry_msgs
)

## Install executables
install(TARGETS
  disturbance_applier_node
  test_disturbance_client
  DESTINATION lib/${PROJECT_NAME}
)

## Install Python scripts
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(PROGRAMS
    scripts/test_disturbance_client.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

## Install launch files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

## Install config files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

## Install source directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src)
  install(DIRECTORY src/
    DESTINATION share/${PROJECT_NAME}/src
  )
endif()

ament_package()
