#!/usr/bin/env python3
"""
ROS 2 Launch file for visualizing the Braco robot in RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch.conditions


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('braco_description')
    
    # Paths to files
    default_model_path = os.path.join(pkg_share, 'urdf', 'Braço.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'launch', 'urdf.rviz')
    
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui (set false when using trajectory maker)'
    )
    
    # Get launch configuration
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    gui = LaunchConfiguration('gui')
    
    # Robot State Publisher - publishes transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', model]),
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher GUI - allows manual joint control
    # Only launched if gui:=true
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(gui)
    )
    
    # RViz2 - visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        model_arg,
        rviz_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
