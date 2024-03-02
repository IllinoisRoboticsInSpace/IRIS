import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist

from sensor_msgs.msg import Joy

from std_msgs.msg import Bool, Float32

import serial

class gamepad_node(Node):

    def __init__(self):
        super().__init__('gamepad_node')

        # subscribes to 'joy' topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info(f"Created node {self.get_name()}")

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

        self.AUTO_MODE = 7
        self.STOP_MODE = 8

        self.prev_state = [0.0] * 9 + [False] * 0
        self.curr_state = [0.0] * 9 + [False] * 0

        # publishes to 'gamepad' topic
        self.gamepad_publishers = [None] * 9

        self.gamepad_publishers[self.LEFT_DRIVE] = self.create_publisher(Float32, '/gamepad/left_drive', 10)
        self.gamepad_publishers[self.RIGHT_DRIVE] = self.create_publisher(Float32, '/gamepad/right_drive', 10)

        self.gamepad_publishers[self.LEFT_BACK_COLL] = self.create_publisher(Bool, '/gamepad/left_back_coll', 10)
        self.gamepad_publishers[self.RIGHT_BACK_COLL] = self.create_publisher(Bool, '/gamepad/right_back_coll', 10)
        self.gamepad_publishers[self.EXC_INTERNAL] = self.create_publisher(Bool, '/gamepad/exc_internal', 10)    
        self.gamepad_publishers[self.EXC_THREAD_ROD] = self.create_publisher(Bool, '/gamepad/exc_thread_rod', 10)
        self.gamepad_publishers[self.EXC_PIVOT_LIN] = self.create_publisher(Bool, '/gamepad/exc_pivot_lin', 10)
        self.gamepad_publishers[self.AUTO_MODE] = self.create_publisher(Bool, '/gamepad/auto_mode', 10)

        self.gamepad_publishers[self.STOP_MODE] = self.create_publisher(Bool, '/gamepad/stop_mode', 10)
        

    def joy_callback(self, joy_msg: Joy):

        if (joy_msg.buttons[8] == 1):
            self.curr_state[self.STOP_MODE] = Bool(True)
            self.gamepad_publishers[self.STOP_MODE].publish(self.curr_state[self.STOP_MODE])

        if (joy_msg.buttons[self.AUTO_MODE] == 1):
            self.curr_state[self.AUTO_MODE] = Bool(not self.curr_state[self.AUTO_MODE])

        # BEHAVIOR?
        # if self.stop:
        #     for i in range(len(self.curr_state) - 1):
        #         self.curr_state[i] = Float32(0.0)

        else:
            # indexed according to above
            self.get_logger().info(f"joy msg axes 1 type: {type(Float32(joy_msg.axes[1]))}")
            self.curr_state[self.LEFT_DRIVE] = Float32(joy_msg.axes[1])
            self.curr_state[self.RIGHT_DRIVE] = Float32(joy_msg.axes[4])
            self.curr_state[self.LEFT_BACK_COLL] = Bool(joy_msg.buttons[4])
            self.curr_state[self.RIGHT_BACK_COLL] = Bool(joy_msg.buttons[5])
            self.curr_state[self.EXC_INTERNAL] = Bool(joy_msg.buttons[0])
            self.curr_state[self.EXC_THREAD_ROD] = Bool(joy_msg.buttons[1])
            self.curr_state[self.EXC_PIVOT_LIN] = Bool(joy_msg.buttons[2])

        for i in range(len(self.curr_state)):
            if (self.prev_state[i] != self.curr_state[i]):
                self.prev_state[i] = self.curr_state[i]
                self.gamepad_publishers[i].publish(self.curr_state[i])
        
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
