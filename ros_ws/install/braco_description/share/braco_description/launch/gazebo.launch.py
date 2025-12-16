#!/usr/bin/env python3
"""
ROS 2 Launch file for spawning the Braco robot in Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('braco_description')
    
    # Paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'braco.xacro')
    world_file = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')
    
    # Set Gazebo model path to include our package
    gazebo_model_path = os.path.join(pkg_share, '..')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start Gazebo server
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    # Start Gazebo client
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', default_model_path]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Spawn robot in Gazebo with delay
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'braco_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_entity
    ])
