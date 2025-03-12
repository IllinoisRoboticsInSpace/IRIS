"""
@file robot_controller_launch.py
@author Rushil Shah

@brief Launch File to Control Robot from Joystick
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tank_control', executable='joy_to_tank_control',
            name='gamepad_to_sparkmax', parameters=[],
            output='screen',
            emulate_tty=True,
        )
    ])
