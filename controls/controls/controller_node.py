import rclpy, copy
from dataclasses import dataclass
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
 
# `Joy.buttons` and `Joy.axes` indices for an Xbox 360 Controller:
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
BUTTON_BACK = 6
BUTTON_START = 7
BUTTON_GUIDE = 8
BUTTON_LEFT_STICK = 9
BUTTON_RIGHT_STICK = 10
BUTTON_MAX = 11

LEFT_STICK_X = 0
LEFT_STICK_Y = 1
LEFT_TRIGGER = 2
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5
DPAD_X = 6
DPAD_Y = 7

def joystick_input_conversion(val):
    return (val * 0.75) ** 3

def trigger_input_conversion(val):
    return (1 - val) / 2

@dataclass
class InputState:
    left_drive: float
    right_drive: float
    raise_scooper: bool
    lower_scooper: bool
    raise_dumper: bool
    lower_dumper: bool
    # Sticky.
    run_excavate_routine: bool
    run_deposit_routine: bool
    run_traversal_routine: bool
    autonomous_flag: bool
    stop_flag: bool

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.tread_speed_left_teleop_publisher = self.create_publisher(Float32, 'tread_speed_left_teleop', 10)
        self.tread_speed_right_teleop_publisher = self.create_publisher(Float32, 'tread_speed_right_teleop', 10)
        self.raise_scooper_teleop_publisher = self.create_publisher(Bool, 'raise_scooper_teleop', 10)
        self.lower_scooper_teleop_publisher = self.create_publisher(Bool, 'lower_scooper_teleop', 10)
        self.raise_dumper_teleop_publisher = self.create_publisher(Bool, 'raise_dumper_teleop', 10)
        self.lower_dumper_teleop_publisher = self.create_publisher(Bool, 'lower_dumper_teleop', 10)
        self.exc_routine_publisher = self.create_publisher(Bool, 'exc_routine', 10)
        self.dep_routine_publisher = self.create_publisher(Bool, 'dep_routine', 10)
        self.tra_routine_publisher = self.create_publisher(Bool, 'tra_routine', 10)
        self.auto_flag_teleop_publisher = self.create_publisher(Bool, 'auto_flag_teleop', 10)
        self.stop_flag_teleop_publisher = self.create_publisher(Bool, 'stop_flag_teleop', 10)

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_response, 10)
        self.exc_routine_subscription = self.create_subscription(Bool, 'exc_routine', self.exc_routine_response, 10)
        self.dep_routine_subscription = self.create_subscription(Bool, 'dep_routine', self.dep_routine_response, 10)
        self.tra_routine_subscription = self.create_subscription(Bool, 'tra_routine', self.tra_routine_response, 10)

        self.prev_buttons = [0] * BUTTON_MAX
        
        self.curr_state = InputState(0.0, 0.0, False, False, False, False, False, False, False, False, False)
        self.prev_state = InputState(0.0, 0.0, False, False, False, False, False, False, False, False, False)
    
    def exc_routine_response(self, message: Bool):
        self.prev_state.run_excavate_routine = message.data
    
    def dep_routine_response(self, message: Bool):
        self.prev_state.run_excavate_routine = message.data
    
    def tra_routine_response(self, message: Bool):
        self.prev_state.run_excavate_routine = message.data
    
    def joy_response(self, joy_msg: Joy):
        # The `stop_flag_teleop` and `auto_flag_teleop` state variables need to toggle on button presses.
        if self.prev_buttons[BUTTON_GUIDE] == 0 and joy_msg.buttons[BUTTON_GUIDE] == 1:
            self.curr_state.stop_flag = not self.curr_state.stop_flag
        
        if self.prev_buttons[BUTTON_X] == 0 and joy_msg.buttons[BUTTON_X] == 1:
            self.curr_state.autonomous_flag = not self.curr_state.autonomous_flag
        
        if not self.curr_state.stop_flag:
            if not self.curr_state.autonomous_flag:
                self.curr_state.left_drive = joystick_input_conversion(joy_msg.axes[LEFT_STICK_Y])
                self.curr_state.right_drive = joystick_input_conversion(joy_msg.axes[RIGHT_STICK_Y])

                # If any combination of the buttons are pressed at the same time, ignore.
                if not ((self.prev_buttons[BUTTON_A] == 0 and joy_msg.buttons[BUTTON_A] == 1 and self.prev_buttons[BUTTON_B] == 0 and joy_msg.buttons[BUTTON_B] == 1) or
                        (self.prev_buttons[BUTTON_A] == 0 and joy_msg.buttons[BUTTON_A] == 1 and self.prev_buttons[BUTTON_Y] == 0 and joy_msg.buttons[BUTTON_Y] == 1) or
                        (self.prev_buttons[BUTTON_B] == 0 and joy_msg.buttons[BUTTON_B] == 1 and self.prev_buttons[BUTTON_Y] == 0 and joy_msg.buttons[BUTTON_Y] == 1)):
                    
                    if self.prev_buttons[BUTTON_A] == 0 and joy_msg.buttons[BUTTON_A] == 1:
                        self.curr_state.run_excavate_routine = not self.curr_state.run_excavate_routine
                        if self.curr_state.run_excavate_routine:
                            self.curr_state.run_deposit_routine = False
                            self.curr_state.run_traversal_routine = False
                    
                    if self.prev_buttons[BUTTON_B] == 0 and joy_msg.buttons[BUTTON_B] == 1:
                        self.curr_state.run_deposit_routine = not self.curr_state.run_deposit_routine
                        if self.curr_state.run_deposit_routine:
                            self.curr_state.run_excavate_routine = False
                            self.curr_state.run_traversal_routine = False
                    
                    if self.prev_buttons[BUTTON_Y] == 0 and joy_msg.buttons[BUTTON_Y] == 1:
                        self.curr_state.run_traversal_routine = not self.curr_state.run_traversal_routine
                        if self.curr_state.run_traversal_routine:
                            self.curr_state.run_excavate_routine = False
                            self.curr_state.run_deposit_routine = False

                trigger_left = trigger_input_conversion(joy_msg.axes[LEFT_TRIGGER]) > 0.7
                trigger_right = trigger_input_conversion(joy_msg.axes[RIGHT_TRIGGER]) > 0.7

                # If both LT and RT are pressed, we need to just ignore this.
                if not (trigger_left and trigger_right):
                    self.curr_state.raise_scooper = trigger_left
                    self.curr_state.lower_scooper = trigger_right
                else:
                    self.curr_state.raise_scooper = False
                    self.curr_state.lower_scooper = False

                # If both LB and RB are pressed, we need to just ignore this.
                if not (joy_msg.buttons[LEFT_BUMPER] == 1 and joy_msg.buttons[RIGHT_BUMPER] == 1):
                    self.curr_state.lower_dumper = joy_msg.buttons[LEFT_BUMPER] == 1
                    self.curr_state.raise_dumper = joy_msg.buttons[RIGHT_BUMPER] == 1
                else:
                    self.curr_state.lower_dumper = False
                    self.curr_state.raise_dumper = False

            else:
                # If the `auto_flag_teleop` flag is active, we want to assert the manual control state variables to their defaults.
                self.curr_state.left_drive = 0.0
                self.curr_state.right_drive = 0.0
                self.curr_state.raise_scooper = False
                self.curr_state.lower_scooper = False
                self.curr_state.lower_dumper = False
                self.curr_state.raise_dumper = False
                
                # If any combination of the buttons are pressed at the same time, ignore.
                if not ((self.prev_buttons[BUTTON_A] == 0 and joy_msg.buttons[BUTTON_A] == 1 and self.prev_buttons[BUTTON_B] == 0 and joy_msg.buttons[BUTTON_B] == 1) or
                        (self.prev_buttons[BUTTON_A] == 0 and joy_msg.buttons[BUTTON_A] == 1 and self.prev_buttons[BUTTON_Y] == 0 and joy_msg.buttons[BUTTON_Y] == 1) or
                        (self.prev_buttons[BUTTON_B] == 0 and joy_msg.buttons[BUTTON_B] == 1 and self.prev_buttons[BUTTON_Y] == 0 and joy_msg.buttons[BUTTON_Y] == 1)):
                    
                    if self.prev_buttons[BUTTON_A] == 0 and joy_msg.buttons[BUTTON_A] == 1:
                        self.curr_state.run_excavate_routine = True
                        self.curr_state.run_deposit_routine = False
                        self.curr_state.run_traversal_routine = False
                    
                    if self.prev_buttons[BUTTON_B] == 0 and joy_msg.buttons[BUTTON_B] == 1:
                        self.curr_state.run_deposit_routine = True
                        self.curr_state.run_excavate_routine = False
                        self.curr_state.run_traversal_routine = False
                    
                    if self.prev_buttons[BUTTON_Y] == 0 and joy_msg.buttons[BUTTON_Y] == 1:
                        self.curr_state.run_traversal_routine = True
                        self.curr_state.run_excavate_routine = False
                        self.curr_state.run_deposit_routine = False
        else:
            # If the `stop_flag_teleop` flag is active, we want to assert all state variables to their defaults.
            self.curr_state.left_drive = 0.0
            self.curr_state.right_drive = 0.0
            self.curr_state.raise_scooper = False
            self.curr_state.lower_scooper = False
            self.curr_state.lower_dumper = False
            self.curr_state.raise_dumper = False

            #finish
            self.curr_state.exc_routine = False

        # Update all publishers with the current state variables, if there was a change from the previous value.

        if self.prev_state.left_drive != self.curr_state.left_drive:
            self.tread_speed_left_teleop_publisher.publish(Float32(data=self.curr_state.left_drive))
        
        if self.prev_state.right_drive != self.curr_state.right_drive:
            self.tread_speed_right_teleop_publisher.publish(Float32(data=self.curr_state.right_drive))
        
        if self.prev_state.raise_scooper != self.curr_state.raise_scooper:
            self.raise_scooper_teleop_publisher.publish(Bool(data=self.curr_state.raise_scooper))
        
        if self.prev_state.lower_scooper != self.curr_state.lower_scooper:
            self.lower_scooper_teleop_publisher.publish(Bool(data=self.curr_state.lower_scooper))
        
        if self.prev_state.raise_dumper != self.curr_state.raise_dumper:
            self.raise_dumper_teleop_publisher.publish(Bool(data=self.curr_state.raise_dumper))

        if self.prev_state.lower_dumper != self.curr_state.lower_dumper:
            self.lower_dumper_teleop_publisher.publish(Bool(data=self.curr_state.lower_dumper))
        
        if self.prev_state.run_excavate_routine != self.curr_state.run_excavate_routine:
            self.exc_routine_publisher.publish(Bool(data=self.curr_state.run_excavate_routine))

        if self.prev_state.run_deposit_routine != self.curr_state.run_deposit_routine:
            self.dep_routine_publisher.publish(Bool(data=self.curr_state.run_deposit_routine))
        
        if self.prev_state.run_traversal_routine != self.curr_state.run_traversal_routine:
            self.tra_routine_publisher.publish(Bool(data=self.curr_state.run_traversal_routine))

        if self.prev_state.autonomous_flag != self.curr_state.autonomous_flag:
            self.auto_flag_teleop_publisher.publish(Bool(data=self.curr_state.autonomous_flag))

        if self.prev_state.stop_flag != self.curr_state.stop_flag:
            self.stop_flag_teleop_publisher.publish(Bool(data=self.curr_state.stop_flag))
        
        # Update `self.prev_state`. We need to force a copy here to prevent `self.prev_state` from referencing `self.curr_state`.
        self.prev_state = copy.deepcopy(self.curr_state)
        self.prev_buttons = copy.deepcopy(joy_msg.buttons)

def main(args=None):
    rclpy.init(args=args)

    gamepad = Controller()
    try:
        rclpy.spin(gamepad)
    except KeyboardInterrupt:
        pass
    finally:
        gamepad.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()