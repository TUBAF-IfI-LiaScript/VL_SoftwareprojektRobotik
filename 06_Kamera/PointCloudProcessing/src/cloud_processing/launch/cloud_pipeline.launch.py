# launch/cloud_pipeline.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='cloud_processing',
            executable='filter_node',
            name='filter_node',
            parameters=[  # <-- hier eine Liste verwenden!
                {
                    'min_dist': 0.5,
                    'max_dist': 8.0,
                    'input_topic': '/zed2i_front/zed_node_front/point_cloud/cloud_registered'
                }
            ],
            output='screen',
        ),

        Node(
            package='cloud_processing',
            executable='ground_detection_node',
            name='ground_detection_node',
            parameters=[
                {
                    'ground_threshold': 0.05,
                    'min_ground_points': 100
                }
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('cloud_processing'),
                'rviz',
                'cloud_pipeline.rviz'
            )]
        ),
    ])
