import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist

from sensor_msgs.msg import Joy

from std_msgs.msg import String

import serial

class gamepad_node(Node):

    def __init__(self):
        super().__init__('gamepad_node')

        # subscribes to 'joy' topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info(f"Created node {self.get_name()}")

        # publishes to 'gamepad' topic
        self.publisher_ = self.create_publisher(String, '/gamepad', 10)

        self.input_scale = 80
        self.noise_allowance = 0.5

        # Motor ID:
        # * 0: Left Drive
        # * 1: Right Drive
        # * 2: Left Back Collection Motor
        # * 3: Right Back Collection Motor
        # * 4: Excavator Internal Motor
        # * 5: Excavator Threaded Rod Actuator
        # * 6: Excavator Pivoting Linear Actuator

        self.LEFT_DRIVE = 0
        self.RIGHT_DRIVE = 1
        self.LEFT_BACK_COLL = 2
        self.RIGHT_BACK_COLL = 3
        self.EXC_INTERNAL = 4
        self.EXC_THREAD_ROD = 5
        self.EXC_PIVOT_LIN = 6

        self.stop = False

        self.prev_state = [0,0,0,0,0,0]
        self.curr_state = [0,0,0,0,0,0]

    def joy_callback(self, joy_msg: Joy):

        if self.stop | (joy_msg.buttons[8] == 1):
            self.stop = True
            for i in range(len(self.curr_state)):
                self.curr_state[i] = 0

        else:
            self.curr_state[self.LEFT_DRIVE] = joy_msg.axes[1]
            self.curr_state[self.RIGHT_DRIVE] = joy_msg.axes[4]
            self.curr_state[self.LEFT_BACK_COLL] = joy_msg.buttons[4]
            self.curr_state[self.RIGHT_BACK_COLL] = joy_msg.buttons[5]
            self.curr_state[self.EXC_INTERNAL] = joy_msg.buttons[0]
            self.curr_state[self.EXC_THREAD_ROD] = joy_msg.buttons[1]
            self.curr_state[self.EXC_PIVOT_LIN] = joy_msg.buttons[2]

        for i in range(len(self.curr_state)):
            if (self.prev_state[i] != self.curr_state[i]):
                self.prev_state[i] = self.curr_state[i]

                #! TEMPORARY
                returnMsg = String()
                returnMsg.data = 'power = ' + str(self.curr_state[i])
                self.publisher_.publish(returnMsg)


def main(args=None):
    rclpy.init(args=args)

    gamepad = gamepad_node()
    try:
        rclpy.spin(gamepad)
    except KeyboardInterrupt:
        pass
    finally:
        gamepad.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
