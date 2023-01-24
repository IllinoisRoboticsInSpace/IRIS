import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='jetson', executable='twist_serial',
            name='twist_serial')
    ])
