from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction
def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'py_pubsub', 'campub'],
            name='camfeed'
        ),
        TimerAction(
            period= 1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'camsub'],
                    name = 'camsub'
                )
            ]
        )

    ])