import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='my_tutorial_package', node_executable='heart_beat', output='screen')
                ], period = 5.0
        )
    ])
