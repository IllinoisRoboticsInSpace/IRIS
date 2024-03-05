import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32

# Motor ID:
# * 0: Left Drive -> left joystick
# * 1: Right Drive -> right joystick
# * 2: Left Back Collection Motor -> LB
# * 3: Right Back Collection Motor -> RB
# * 4: Excavator Internal Motor -> A
# * 5: Excavator Threaded Rod Actuator -> B
# * 6: Excavator Pivoting Linear Actuator -> X
# * 7: Auto Mode -> Y
# * 8: Stop Mode -> Xbox button

LEFT_DRIVE = 0
RIGHT_DRIVE = 1
LEFT_BACK_COLL = 2
RIGHT_BACK_COLL = 3
EXC_INTERNAL = 4
EXC_THREAD_ROD = 5
EXC_PIVOT_LIN = 6

AUTO_MODE = 7
STOP_MODE = 8

class gamepad_node(Node):

    def __init__(self):
        super().__init__('gamepad_node')

        # subscribes to 'joy' topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Check if arduino node has recieved stop message so we can stop sending it, need to see how to get 2 nodes in callback
        # self.stop_receive = self.create_subscription(Bool, '/arduino_comms/stop_received', self.joy_callback, 10)

        self.get_logger().info(f"Created node {self.get_name()}")
        self.auto_button_prev_state = False
        self.stop = False

        # self.software_scale = 80
        # self.software_deadzone = 0.5

        self.prev_state = [0.0] * 2 + [False] * 7
        self.curr_state = [0.0] * 2 + [False] * 7

        # publishes to 'gamepad' topic
        self.gamepad_publishers = [None] * 9

        self.gamepad_publishers[LEFT_DRIVE] = self.create_publisher(Float32, '/gamepad/left_drive', 10)
        self.gamepad_publishers[RIGHT_DRIVE] = self.create_publisher(Float32, '/gamepad/right_drive', 10)
        self.gamepad_publishers[LEFT_BACK_COLL] = self.create_publisher(Bool, '/gamepad/left_back_coll', 10)
        self.gamepad_publishers[RIGHT_BACK_COLL] = self.create_publisher(Bool, '/gamepad/right_back_coll', 10)
        self.gamepad_publishers[EXC_INTERNAL] = self.create_publisher(Bool, '/gamepad/exc_internal', 10)    
        self.gamepad_publishers[EXC_THREAD_ROD] = self.create_publisher(Bool, '/gamepad/exc_thread_rod', 10)
        self.gamepad_publishers[EXC_PIVOT_LIN] = self.create_publisher(Bool, '/gamepad/exc_pivot_lin', 10)

        self.gamepad_publishers[AUTO_MODE] = self.create_publisher(Bool, '/gamepad/auto_mode', 10)
        self.gamepad_publishers[STOP_MODE] = self.create_publisher(Bool, '/gamepad/stop_mode', 10)
        

    def joy_callback(self, joy_msg: Joy):

        if (joy_msg.buttons[8] == 1):
            self.curr_state[self.STOP_MODE] = True


        if (joy_msg.buttons[AUTO_MODE] == 1 and self.auto_button_prev_state == False):
            self.curr_state[AUTO_MODE] = not self.curr_state[AUTO_MODE]
            self.auto_button_prev_state = True

        if self.stop:
            for i in range(len(self.curr_state) - 1):
                self.curr_state[i] = 0

        else:
            # indexed according to above
            self.curr_state[LEFT_DRIVE] = joy_msg.axes[1]
            self.curr_state[RIGHT_DRIVE] = joy_msg.axes[4]
            self.curr_state[LEFT_BACK_COLL] = joy_msg.buttons[4]
            self.curr_state[RIGHT_BACK_COLL] = joy_msg.buttons[5]
            self.curr_state[EXC_INTERNAL] = joy_msg.buttons[0]
            self.curr_state[EXC_THREAD_ROD] = joy_msg.buttons[1]
            self.curr_state[EXC_PIVOT_LIN] = joy_msg.buttons[2]
            self.auto_button_prev_state = joy_msg.buttons[AUTO_MODE]

        for i in range(len(self.curr_state)):
            if (self.prev_state[i] != self.curr_state[i]):
                self.prev_state[i] = self.curr_state[i]

                if i < 2:
                    pub_msg = Float32()
                    pub_msg.data = float(self.curr_state[i])
                    
                else:
                    pub_msg = Bool()
                    pub_msg.data = bool(self.curr_state[i])
                
                self.get_logger().info(f"index: {i}, value: {self.curr_state[i]}")
                self.gamepad_publishers[i].publish(pub_msg)
        
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
