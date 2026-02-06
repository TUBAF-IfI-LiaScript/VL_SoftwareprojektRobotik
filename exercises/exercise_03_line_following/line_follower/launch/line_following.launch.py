#!/usr/bin/env python3
"""
Launch-File für das Linienverfolgungssystem

Startet alle drei Nodes:
- line_detector_node: Erkennt die weiße Linie
- controller_node: PID-Regler für Lenkung
- obstacle_detector_node: Hinderniserkennung

Verwendung:
    ros2 launch line_follower line_following.launch.py

Mit Parametern:
    ros2 launch line_follower line_following.launch.py kp:=0.8 linear_velocity:=0.15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paketverzeichnis finden
    pkg_dir = get_package_share_directory('line_follower')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Launch-Argumente für schnelles Tuning
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='0.5',
        description='PID Proportional-Verstärkung'
    )

    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='0.0',
        description='PID Integral-Verstärkung'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.1',
        description='PID Differential-Verstärkung'
    )

    linear_vel_arg = DeclareLaunchArgument(
        'linear_velocity',
        default_value='0.1',
        description='Vorwärtsgeschwindigkeit [m/s]'
    )

    # Nodes
    line_detector_node = Node(
        package='line_follower',
        executable='line_detector_node',
        name='line_detector',
        parameters=[params_file],
        output='screen'
    )

    obstacle_detector_node = Node(
        package='line_follower',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        parameters=[params_file],
        output='screen'
    )

    controller_node = Node(
        package='line_follower',
        executable='controller_node',
        name='controller',
        parameters=[
            params_file,
            {
                'kp': LaunchConfiguration('kp'),
                'ki': LaunchConfiguration('ki'),
                'kd': LaunchConfiguration('kd'),
                'linear_velocity': LaunchConfiguration('linear_velocity'),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch-Argumente
        kp_arg,
        ki_arg,
        kd_arg,
        linear_vel_arg,

        # Nodes
        line_detector_node,
        obstacle_detector_node,
        controller_node,
    ])
