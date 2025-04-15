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

# real vals
N_AXIS_USED = 2

LEFT_DRIVE = 0
RIGHT_DRIVE = 1

# boolean vals
N_BUTTONS_USED = 7

LEFT_BACK_COLL = 2
RIGHT_BACK_COLL = 3
EXC_INTERNAL = 4
EXC_THREAD_ROD = 5
EXC_PIVOT_LIN = 6

AUTO_MODE = 7
STOP_MODE = 8

def get_gamepad_mapping():
    mapping = {"left_drive": LEFT_DRIVE, "right_drive": RIGHT_DRIVE, "left_back_cltn_mtr": LEFT_BACK_COLL, 
                                "right_back_cltn_mtr": RIGHT_BACK_COLL, "exc_intrnl_mtr": EXC_INTERNAL, "exc_thrd_rod_actr": EXC_THREAD_ROD,
                                "exc_pvt_lin_actr": EXC_PIVOT_LIN, "auto_mode": AUTO_MODE, "stop_mode": STOP_MODE}
    
    return mapping

def get_inverse_gamepad_mapping():
    orig_mapping = get_gamepad_mapping()
    new_mapping = {v: k for k, v in orig_mapping.items()}
    return new_mapping

class gamepad_node(Node):

    def __init__(self):
        super().__init__('gamepad_node')

        self.gamepad_map = get_gamepad_mapping()
        self.inv_gamepad_map = get_inverse_gamepad_mapping()

        # subscribes to 'joy' topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Check if arduino node has recieved stop message so we can stop sending it, need to see how to get 2 nodes in callback
        # self.stop_receive = self.create_subscription(Bool, '/arduino_comms/stop_received', self.joy_callback, 10)

        self.get_logger().info(f"Created node {self.get_name()}")

        # self.software_scale = 80
        # self.software_deadzone = 0.5

        self.prev_state = [0.0] * N_AXIS_USED + [False] * N_BUTTONS_USED
        self.curr_state = [0.0] * N_AXIS_USED + [False] * N_BUTTONS_USED
        # self.get_logger().info(f"prev: {self.prev_state}")
        # self.get_logger().info(f"prev: {self.curr_state}")
        

        # publisher init to 'gamepad' topic
        self.gamepad_publishers = [None] * (N_AXIS_USED + N_BUTTONS_USED)

        for key, value in self.gamepad_map.items():
            if key in ["left_drive", "right_drive"]:
                msg_type = Float32
            else:
                msg_type = Bool

            self.gamepad_publishers[value] = self.create_publisher(msg_type, f"/gamepad/{key}", 10)

        self.joystick_button_mapping = {"A": 0, "B": 1, "X": 2, "Y": 3, "LB": 4,
                                 "RB": 5, "back": 6, "start": 7, "power": 8,
                                 "button stick left": 9, "button stick right": 10}
        self.joystick_axis_mapping = {"LR axis stick left": 0, 
                                      "UD axis stick left": 1,
                                      "LT": 2,
                                      "LR axis stick right": 3,
                                      "UD axis stick right": 4,
                                      "RT": 5,
                                      "cross key LR": 6,
                                      "cross key UD": 7}
        

    def joy_callback(self, joy_msg: Joy):

        # power off
        if (joy_msg.buttons[self.joystick_button_mapping["power"]] == 1):
            self.prev_state[STOP_MODE] = self.curr_state[STOP_MODE]
            self.curr_state[STOP_MODE] = True

        # auto mode
        if (joy_msg.buttons[self.joystick_button_mapping["start"]] == 1):
            self.curr_state[AUTO_MODE] = not self.prev_state[AUTO_MODE]
            self.prev_state[AUTO_MODE] = not self.curr_state[AUTO_MODE]

        if self.curr_state[STOP_MODE] == True:
            for i in range(len(self.curr_state)):
                if i != STOP_MODE:
                    self.curr_state[i] = 0
        else:
            self.curr_state[LEFT_DRIVE] = joy_msg.axes[self.joystick_axis_mapping["UD axis stick left"]]
            self.curr_state[RIGHT_DRIVE] = joy_msg.axes[self.joystick_axis_mapping["UD axis stick right"]]
            self.curr_state[LEFT_BACK_COLL] = joy_msg.buttons[self.joystick_button_mapping["LB"]]
            self.curr_state[RIGHT_BACK_COLL] = joy_msg.buttons[self.joystick_button_mapping["RB"]]
            self.curr_state[EXC_INTERNAL] = joy_msg.buttons[self.joystick_button_mapping["A"]]
            self.curr_state[EXC_THREAD_ROD] = joy_msg.buttons[self.joystick_button_mapping["B"]]
            self.curr_state[EXC_PIVOT_LIN] = joy_msg.buttons[self.joystick_button_mapping["X"]]

        for i in range(len(self.curr_state)):
            if (self.prev_state[i] != self.curr_state[i]):
                self.prev_state[i] = self.curr_state[i]

                if i < N_AXIS_USED:
                    pub_msg = Float32()
                    pub_msg.data = float(self.curr_state[i])

                    if (i == LEFT_DRIVE or i == RIGHT_DRIVE):
                        pub_msg.data = pub_msg.data * abs(pub_msg.data)
                    
                else:
                    pub_msg = Bool()
                    pub_msg.data = bool(self.curr_state[i])
                
                self.get_logger().info(f"{self.inv_gamepad_map[i]}: {self.curr_state[i]}")
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
