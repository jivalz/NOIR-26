from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'py_pubsub', 'esp32rst' ],
            name='esp32_reset'
        ),
        TimerAction(
            period= 1.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'cpp_pubsub', 'ps4'],
                    name = 'ps4_teleop'
                )
            ]
        ),
        TimerAction(
            period= 2.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'camfinal'],
                    name = 'camfeed_final'
                )
            ]
        ),
        TimerAction(
            period= 3.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
                    name = 'micro_ros_agent_0'
                )
            ]
        )

    ])