"""
Face Detection Launch File

Startet:
- v4l2_camera: Kamera-Node für USB-Kameras
- face_detector_node: Gesichtserkennung
- rqt_image_view: Visualisierung der Ergebnisse
- rosbag2: Aufzeichnung der Daten

Verwendung:
    ros2 launch face_detector face_detection.launch.py

Optional Parameter:
    ros2 launch face_detector face_detection.launch.py video_device:=/dev/video2
    ros2 launch face_detector face_detection.launch.py record_bag:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from datetime import datetime


def generate_launch_description():
    """Generiert Launch Description"""

    # Launch-Argumente deklarieren
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path (e.g., /dev/video0)'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='320',  # Reduziert für Performance
        description='Image width in pixels'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='240',  # Reduziert für Performance
        description='Image height in pixels'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Record rosbag (true/false)'
    )

    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Show image viewer (true/false)'
    )

    # Launch-Konfigurationen
    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    record_bag = LaunchConfiguration('record_bag')
    show_image = LaunchConfiguration('show_image')

    # 1. v4l2_camera Node - Kamera-Treiber
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': video_device,
            'image_size': [image_width, image_height],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
        }],
        output='screen',
        emulate_tty=True,
    )

    # 2. Face Detector Node - Gesichtserkennung
    face_detector_node = Node(
        package='face_detector',
        executable='face_detector_node',
        name='face_detector',
        parameters=[{
            'scale_factor': 1.2,  # Größer = schneller (weniger Stufen)
            'min_neighbors': 4,   # Weniger = schneller (aber mehr False Positives)
            'min_size_width': 40,  # Größer = schneller (kleinere Gesichter ignoriert)
            'min_size_height': 40,
        }],
        output='screen',
        emulate_tty=True,
    )

    # 3. rqt_image_view - Bildvisualisierung
    image_viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=['/face_detection/image'],
        condition=IfCondition(show_image),
        output='screen',
    )

    # 4. rosbag2 record - Datenaufzeichnung
    # Erzeuge Timestamp für Bag-Dateinamen
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_filename = f'face_detection_{timestamp}'

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_filename,
            '/image_raw',
            '/face_detection/image',
            '/face_detection/faces',
            '--storage', 'mcap',
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    # Launch Description zusammenstellen
    return LaunchDescription([
        # Argumente
        video_device_arg,
        image_width_arg,
        image_height_arg,
        record_bag_arg,
        show_image_arg,

        # Nodes
        v4l2_camera_node,
        face_detector_node,
        image_viewer_node,
        rosbag_record,
    ])
