"""P 2: Kamerabild entzerren und in die Vogelperspektive umrechnen.

    ros2 launch verkehrswelt bildkorrektur.launch.py kalibrierdatei:=$HOME/kalibrierung/ost.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('verkehrswelt'), 'config', 'homographie.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('kalibrierdatei', description='ost.yaml aus camera_calibration'),
        DeclareLaunchArgument('homographie', default_value=config, description='Ergebnis von boden_kalibrieren'),
        DeclareLaunchArgument('komprimiert', default_value='true', description='false für Rohbilder, z.B. die Aufnahme aus P 1'),
        Node(package='verkehrswelt', executable='image_corrector',
             parameters=[LaunchConfiguration('homographie'),
                         {'kalibrierdatei': LaunchConfiguration('kalibrierdatei'),
                          'komprimiert': ParameterValue(LaunchConfiguration('komprimiert'), value_type=bool)}]),
    ])
