import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy


class TeleopNode(Node):

    def __init__(self):
        super().__init__('TeleopNode')

        # subscribe directly from controller joystick
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10)

        # REMOVE : publishes to velocity command (twist) for the motors
        self.publisher = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # logging thing
        self.get_logger().info(f"Created node {self.get_name()}")

    def joystick_callback(self, msg: Joy):

        # up-down on the joystick
        straight = msg.axes[1]

        # left-right on the joystick
        turn = msg.axes[0]

        # publish the straight-line and turn necessary for robot via twist
        self.publisher.publish(Twist(
            linear=Vector3(x=straight, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=turn)
        ))

    def joystick_twist_callback(self, msg: Twist):

        # open-loop control, callback is nothing but notification
        linear: Vector3 = msg.linear
        angular: Vector3 = msg.angular
        self.get_logger().info(f"Linear: {linear.x:.2f} | Angular: {angular.z:.2f}")


def main(args=None):

    # initialize the ros py object and teleop node objects
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    try:
        # set ros to run the teleop node
        rclpy.spin(teleop_node)
        
    except KeyboardInterrupt:
        pass
    finally:
        # end everything
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
