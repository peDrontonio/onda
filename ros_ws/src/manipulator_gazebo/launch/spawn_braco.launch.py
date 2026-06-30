#!/usr/bin/env python3
"""
Spawn the Braco robot in Gazebo Harmonic with effort-based control.

Run:
    ros2 launch manipulator_gazebo spawn_braco.launch.py

Prerequisites (install once):
    sudo apt install \
        ros-humble-gz-ros2-control \
        ros-humble-effort-controllers \
        ros-humble-joint-state-broadcaster
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_braco = get_package_share_directory('braco_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_xacro = os.path.join(pkg_braco, 'urdf', 'Braço.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Gazebo Harmonic ───────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r empty.sdf'],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro]),
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spawn robot into Gazebo ───────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_braco',
        arguments=[
            '-world', 'empty',
            '-name', 'braco',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )

    # ── Controllers ───────────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller'],
        output='screen',
    )

    # ── Custom PD+G controller node ───────────────────────────────────────────
    braco_controller = Node(
        package='braco_description',
        executable='braco_controller.py',
        name='braco_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Startup sequence: spawn → joint_state_broadcaster → effort_controller → braco_controller
    activate_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    activate_effort_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[effort_controller_spawner],
        )
    )

    activate_braco_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=effort_controller_spawner,
            on_exit=[braco_controller],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start Gazebo GUI'),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        activate_state_broadcaster,
        activate_effort_controller,
        activate_braco_controller,
    ])
