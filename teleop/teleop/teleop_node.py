import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy


class TeleopNode(Node):

    def __init__(self):
        super().__init__('TeleopNode')
        # self.subscription = self.create_subscription(
        #     Twist, '/cmd_vel', self.joystick_twist_callback, 10)  # last param is queue size
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10)

        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.get_logger().info(f"Created node {self.get_name()}")

    def joystick_callback(self, msg: Joy):
        self.publisher.publish(Twist(
            linear=Vector3(x=msg.axes[1], y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=msg.axes[0])
        ))
        #self.get_logger().info(str(msg.axes[1]))

    def joystick_twist_callback(self, msg: Twist):
        linear: Vector3 = msg.linear
        angular: Vector3 = msg.angular
        self.get_logger().info(f"Linear: {linear.x:.2f} | Angular: {angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopNode()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
