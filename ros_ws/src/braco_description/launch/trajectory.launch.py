#!/usr/bin/env python3
"""
ROS 2 Launch file for Braco robot with trajectory control capability.

This launch file starts:
1. Robot State Publisher - publishes robot transforms
2. Trajectory Publisher - publishes joint states and accepts trajectory commands
3. RViz2 - visualization

Usage:
    ros2 launch braco_description trajectory.launch.py
    
To send trajectories:
    ros2 topic pub /trajectory_command trajectory_msgs/msg/JointTrajectory "..."

Author: Pedro Antonio
Date: 2025-12-16
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    demo_arg = DeclareLaunchArgument(
        name='demo',
        default_value='false',
        description='Run demo trajectory on startup'
    )
    
    # Get launch configuration
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    demo = LaunchConfiguration('demo')
    
    # Robot State Publisher - publishes transforms from URDF
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
    
    # Trajectory Publisher Node - publishes joint states and handles trajectory commands
    trajectory_publisher_node = Node(
        package='braco_description',
        executable='trajectory_publisher.py',
        name='trajectory_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
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
        demo_arg,
        robot_state_publisher_node,
        trajectory_publisher_node,
        rviz_node,
    ])
