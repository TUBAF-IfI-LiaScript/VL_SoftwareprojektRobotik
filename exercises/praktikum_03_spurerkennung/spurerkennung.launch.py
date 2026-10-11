"""P 3: Bildkorrektur aus P 2 und Spurerkennung gemeinsam starten.

Kopieren Sie diese Datei nach verkehrswelt/launch/.
    ros2 launch verkehrswelt spurerkennung.launch.py kalibrierdatei:=... homographie:=...

Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    paket = get_package_share_directory('verkehrswelt')
    return LaunchDescription([
        DeclareLaunchArgument('kalibrierdatei'),
        DeclareLaunchArgument('homographie'),
        DeclareLaunchArgument('komprimiert', default_value='true'),
        DeclareLaunchArgument('spurparameter', default_value=os.path.join(paket, 'config', 'spur.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(paket, 'launch', 'bildkorrektur.launch.py')),
            launch_arguments={'kalibrierdatei': LaunchConfiguration('kalibrierdatei'),
                              'homographie': LaunchConfiguration('homographie'),
                              'komprimiert': LaunchConfiguration('komprimiert')}.items()),
        Node(package='verkehrswelt', executable='lane_detector',
             parameters=[LaunchConfiguration('spurparameter')]),
    ])
