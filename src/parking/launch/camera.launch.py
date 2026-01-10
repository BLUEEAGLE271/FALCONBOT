import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parking',
            executable='rectified_parking',
            name='vpi_rectified_node',
            output='screen',
            emulate_tty=True
        )
    ])