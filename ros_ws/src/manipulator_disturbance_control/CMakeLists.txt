cmake_minimum_required(VERSION 3.8)
project(manipulator_disturbance_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(manipulator_msgs REQUIRED)

# Modern Gazebo (Harmonic) transport + messages, replacing Gazebo Classic gazebo_msgs
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
)

# Disturbance applier node executable
add_executable(disturbance_applier_node
  src/disturbance_applier_node.cpp
)

ament_target_dependencies(disturbance_applier_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  manipulator_msgs
)

target_link_libraries(disturbance_applier_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# Test disturbance client executable
add_executable(test_disturbance_client 
  src/test_disturbance_client.cpp
)

ament_target_dependencies(test_disturbance_client
  rclcpp
  manipulator_msgs
)

# Install executables
install(TARGETS
  disturbance_applier_node
  test_disturbance_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/test_disturbance_client.py
  scripts/interactive_trajectory_maker.py
  scripts/trajectory_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
