#!/usr/bin/env python3
"""
ROS 2 Launch file for Interactive Trajectory Maker
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='interactive',
        description='Mode: interactive for command line, execute to run saved trajectory'
    )
    
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value='',
        description='Trajectory file to load (for execute mode)'
    )
    
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='50',
        description='Control loop rate in Hz'
    )
    
    # Interactive Trajectory Maker node
    trajectory_maker_node = Node(
        package='manipulator_disturbance_control',
        executable='interactive_trajectory_maker.py',
        name='interactive_trajectory_maker',
        output='screen',
        emulate_tty=True,  # Important for interactive input
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'rate_hz': LaunchConfiguration('rate_hz'),
        }]
    )
    
    return LaunchDescription([
        mode_arg,
        trajectory_file_arg,
        rate_hz_arg,
        trajectory_maker_node,
    ])
