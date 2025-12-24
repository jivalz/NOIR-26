from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'py_pubsub', 'esp32rst' ],
            name='esp32_reset1'
        ),
        TimerAction(
            period= 0.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'camfinal'],
                    name = 'camfeed_final'
                )
            ]
        ),
        TimerAction(
            period= 1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst1'],
                    name = 'esp32_reset2'
                )
            ]
        ),
        TimerAction(
            period= 2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'cpp_pubsub', '2ps4'],
                    name = '2ps4_node'
                )
            ]
        ),
        TimerAction(
            period= 3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
                    name = 'micro_ros_agent_0'
                )
            ]
        ),
        TimerAction(
            period= 4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB1'],
                    name = 'micro_ros_agent_1'
                )
            ]
        )

    ])
