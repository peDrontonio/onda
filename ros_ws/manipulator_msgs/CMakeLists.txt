cmake_minimum_required(VERSION 3.0.2)
project(manipulator_msgs)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
)

## Declare service files
add_service_files(
  DIRECTORY srv
  FILES
  ApplyDisturbance.srv
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Catkin package macro
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)
