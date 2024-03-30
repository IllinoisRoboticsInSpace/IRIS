import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class CmdVelToWheelVelNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheel_vel')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'wheel_vel',
            10)
        self.wheel_separation = 0.5  # Distance between left and right wheel axles (in meters), adjust according to your robot's actual situation
        self.wheel_radius = 0.15  # Wheel radius (in meters), adjust according to your robot's actual situation

    def listener_callback(self, msg):
        # Calculate left and right wheel speeds from Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate the linear speed of the left and right wheels
        left_wheel_velocity = (linear_x - (self.wheel_separation / 2.0) * angular_z) / self.wheel_radius
        right_wheel_velocity = (linear_x + (self.wheel_separation / 2.0) * angular_z) / self.wheel_radius

        # Create and publish a new message
        wheel_vel_msg = Float32MultiArray()
        wheel_vel_msg.data = [left_wheel_velocity, right_wheel_velocity]
        self.publisher.publish(wheel_vel_msg)
        self.get_logger().info(f'Publishing wheel velocities: Left: {left_wheel_velocity} m/s, Right: {right_wheel_velocity} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheelVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()