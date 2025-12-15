cmake_minimum_required(VERSION 3.0.2)
project(manipulator_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  controller_manager
  robot_state_publisher
  xacro
)

catkin_package()

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

## Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
