from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib
import launch

parameters_file_name = 'hokuyo_config.yml'

def generate_launch_description():
    # get current path and go one level up
    parameters_file_path = str(pathlib.Path(__file__).parents[1])
    parameters_file_path += '/config/' + parameters_file_name
    print(parameters_file_path)
    return LaunchDescription([
           Node(
                package='urg_node',
                executable='urg_node_driver',
                output='screen',
                parameters=[
                    parameters_file_path
                ],
                remappings=[
                    ('/scan', '/robot0/scan'),
                ]
            ),
            launch.actions.ExecuteProcess(
              cmd = ['ros2', 'bag', 'record', '-a'],
              output = 'screen',
            )
    ])
