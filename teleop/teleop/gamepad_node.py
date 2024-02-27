import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist

from sensor_msgs.msg import Joy

from std_msgs.msg import String, Float32MultiArray

import serial

class gamepad_node(Node):

    def __init__(self):
        super().__init__('gamepad_node')

        # subscribes to 'joy' topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info(f"Created node {self.get_name()}")

        # publishes to 'gamepad' topic
        self.publisher_ = self.create_publisher(Float32MultiArray, '/gamepad', 10)

        # self.software_scale = 80
        # self.software_deadzone = 0.5

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

        self.prev_state = Float32MultiArray()
        self.prev_state = [0,0,0,0,0,0]
        self.curr_state = Float32MultiArray()
        self.curr_state = [0,0,0,0,0,0]

    def joy_callback(self, joy_msg: Joy):

        if self.stop | (joy_msg.buttons[8] == 1):
            self.stop = True
            for i in range(len(self.curr_state.data)):
                self.curr_state.data[i] = 0

        else:
            # indexed according to above
            self.curr_state.data[self.LEFT_DRIVE] = float(joy_msg.axes[1])
            self.curr_state.data[self.RIGHT_DRIVE] = float(joy_msg.axes[4])
            self.curr_state.data[self.LEFT_BACK_COLL] = float(joy_msg.buttons[4])
            self.curr_state.data[self.RIGHT_BACK_COLL] = float(joy_msg.buttons[5])
            self.curr_state.data[self.EXC_INTERNAL] = float(joy_msg.buttons[0])
            self.curr_state.data[self.EXC_THREAD_ROD] = float(joy_msg.buttons[1])
            self.curr_state.data[self.EXC_PIVOT_LIN] = float(joy_msg.buttons[2])

        for i in range(len(self.curr_state.data)):
            if (self.prev_state.data[i] != self.curr_state.data[i]):
                self.prev_state.data[i] = self.curr_state.data[i]
        
        self.publisher_.publish(self.curr_state)

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
