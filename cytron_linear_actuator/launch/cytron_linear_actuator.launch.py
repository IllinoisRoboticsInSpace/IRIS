from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino communication'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='9600',
            description='Baud rate for serial communication'
        ),
        
        Node(
            package='cytron_linear_actuator',
            executable='cytron_lin_actr',
            name='cytron_lin_actr',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'baud_rate': LaunchConfiguration('baud_rate')}
            ],
            output='screen'
        )
    ])