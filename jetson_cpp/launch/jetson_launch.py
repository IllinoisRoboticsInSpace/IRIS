import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='jetson_cpp', 
            executable='jetson_sparkmax',
            name='motor_comms_node',
            output='screen',
            emulate_tty=True,
            )
    ])
