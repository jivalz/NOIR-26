from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node1',
            parameters=[{'device_id': 0}],
            remappings=[('joy', '/joy1')],   
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node2',
            parameters=[{'device_id': 1}],
            remappings=[('joy', '/joy2')],  
            output='screen'
        ),
    ])