import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')

    config_filepath = os.path.join(get_package_share_directory('teleop'), 
        'config', 'xbox.config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        # Node(
        #     package='teleop_twist_joy', executable='teleop_node',
        #     name='teleop_twist_joy_node', parameters=[config_filepath]),
        Node(
            package='teleop', executable='teleop_node',
            name='TeleopNode', parameters=[]),
    ])
