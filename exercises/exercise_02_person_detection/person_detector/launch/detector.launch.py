"""
Launch file für den PersonDetector Node.

Verwendung:
    ros2 launch person_detector detector.launch.py
    ros2 launch person_detector detector.launch.py confidence_threshold:=0.7
    ros2 launch person_detector detector.launch.py model_name:=yolov8s.pt
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch-Argumente deklarieren
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence score for YOLO detection (0.0-1.0)'
    )

    model_arg = DeclareLaunchArgument(
        'model_name',
        default_value='yolov8n.pt',
        description='YOLOv8 model name (yolov8n.pt, yolov8s.pt, yolov8m.pt)'
    )

    output_arg = DeclareLaunchArgument(
        'output_csv',
        default_value='results/yolo_detections.csv',
        description='Output CSV file path for YOLO detections'
    )

    save_images_arg = DeclareLaunchArgument(
        'save_images',
        default_value='false',
        description='Save debug images when persons are detected'
    )

    # PersonDetector Node
    detector_node = Node(
        package='person_detector',
        executable='detector_node',
        name='person_detector',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'model_name': LaunchConfiguration('model_name'),
            'output_csv': LaunchConfiguration('output_csv'),
            'save_images': LaunchConfiguration('save_images'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        confidence_arg,
        model_arg,
        output_arg,
        save_images_arg,
        detector_node,
    ])
