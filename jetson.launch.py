from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='excavator',
            executable='excavator_node',
            name='excavator_node',
            output='screen',
        ),
        Node(
            package='depositor',
            executable='depositor_node',
            name='depositor_node',
            output='screen',
        ),
        Node(
            package='driver',
            executable='driver',
            name='driver',
            output='screen',
        ),
    ])
