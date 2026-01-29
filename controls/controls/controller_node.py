import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
 
#button assignments
excavation_routine_button = "A" #temp corresponds to button A
deposit_routine_button = "B" #button B
auto_flag_button = "X" #button X

#number of axis and buttons used
number_axis = 4 #Left y axis, Right y axis, Left trigger, Right trigger
number_buttons = 6 #A button, B button, X button, Left bumper, Right bumper, Power button

#index of state corresponds
left_drive = 0
right_drive = 1
raise_scooper = 2
lower_scooper = 3
lower_dumper = 4
raise_dumper = 5
exc_routine = 6
dep_routine = 7
auto_flag = 8
stop_flag = 9

#equation to convert joystick input to usable speed values
def joystick_input_conversion(val):
    return (val * 0.75) ** 3

def trigger_input_conversion(val):
    return (1 - val) / 2

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.controller_publishers = []
        self.controller_publishers.append(self.create_publisher(Float32, 'tread_speed_left_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Float32, 'tread_speed_right_teleop', 10)) #speed value based on eqn        
        self.controller_publishers.append(self.create_publisher(Bool, 'raise_scooper_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'lower_scooper_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'raise_dumper_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'lower_dumper_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'exc_routine_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'dep_routine_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'auto_flag_teleop', 10))
        self.controller_publishers.append(self.create_publisher(Bool, 'stop_flag_teleop', 10))

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_response, 10)
        
        self.curr_state = [0.0] * number_axis + [False] * number_buttons #makes lists of float values for # of axis, and 4 boolean for buttons
        self.prev_state = [0.0] * number_axis + [False] * number_buttons

        self.joystick_button_mapping = {"A": 0, "B": 1, "X": 2, "Y": 3, "LB": 4,
                                 "RB": 5, "back": 6, "start": 7, "power": 8,
                                 "left joy button": 9, "right joy button": 10}

        self.joystick_axis_mapping = {"left joy x": 0, 
                                      "left joy y": 1,
                                      "LT": 2,
                                      "right joy x": 3,
                                      "right joy y": 4,
                                      "RT": 5,
                                      "arrow pad x": 6,
                                      "arrow pad y": 7}

    def joy_response(self, joy_msg: Joy):

        if (joy_msg.buttons[self.joystick_button_mapping["power"]] == 1):
            self.prev_state[stop_flag] = self.curr_state[stop_flag]
            self.curr_state[stop_flag] = True
        
        if (joy_msg.buttons[self.joystick_button_mapping[auto_flag_button]] == 1):
            self.prev_state[auto_flag] = self.curr_state[auto_flag]
            self.curr_state[auto_flag] = not curr_state[auto_flag]

        if (self.curr_state[stop_flag]):
            for i in range(len(self.curr_state)):
                if i != stop_flag:
                    self.prev_state[i] = self.curr_state[i]
                    self.curr_state[i] = 0
        elif (self.curr_state[auto_flag] == False):
            self.prev_state[left_drive] = self.curr_state[left_drive]
            self.curr_state[left_drive] = joystick_input_conversion(joy_msg.axes[joystick_axis_mapping["left joy y"]])

            self.prev_state[right_drive] = self.curr_state[right_drive]
            self.curr_state[right_drive] = joystick_input_conversion(joy_msg.axes[joystick_axis_mapping["right joy y"]])

            #self.prev_state[exc_routine] = self.curr_state[exc_routine]            
            #self.curr_state[exc_routine] = joy_msg.buttons[joystick_button_mapping[excavation_routine_button]]

            #self.prev_state[dep_routine] = self.curr_state[dep_routine]        
            #self.curr_state[dep_routine] = joy_msg.buttons[joystick_button_mapping[deposit_routine_button]]                  
            
            self.prev_state[exc_routine] = self.curr_state[exc_routine]            
            self.curr_state[exc_routine] = False

            self.prev_state[dep_routine] = self.curr_state[dep_routine]        
            self.curr_state[dep_routine] = False                  
            


            trigger_left = trigger_input_conversion(joy_msg.axes[joystick_axis_mapping["LT"]]) > 0.7
            trigger_right = trigger_input_conversion(joy_msg.axes[joystick_axis_mapping["RT"]]) > 0.7


            if (trigger_left and trigger_right): #if both triggers pressed more than 70% ignore
                self.prev_state[raise_scooper] = self.curr_state[raise_scooper]
                self.prev_state[lower_scooper] = self.curr_state[lower_scooper]
                self.curr_state[raise_scooper] = False
                self.curr_state[lower_scooper] = False
            elif (trigger_right):
                self.prev_state[lower_scooper] = self.curr_state[lower_scooper]
                self.curr_state[lower_scooper] = trigger_right
            elif (trigger_left):
                self.prev_state[raise_scooper] = self.curr_state[raise_scooper]
                self.curr_state[raise_scooper] = trigger_left
            else:
                self.prev_state[raise_scooper] = self.curr_state[raise_scooper]
                self.curr_state[raise_scooper] = trigger_left
                self.prev_state[lower_scooper] = self.curr_state[lower_scooper]
                self.curr_state[lower_scooper] = trigger_right

            if (joy_msg.buttons[joystick_button_mapping["LB"]] and joy_msg.buttons[joystick_button_mapping["RB"]]): #if both pressed ignore
                self.prev_state[raise_dumper] = self.curr_state[raise_dumper]
                self.prev_state[lower_dumper] = self.curr_state[lower_dumper]
                self.curr_state[raise_dumper] = False
                self.curr_state[lower_dumper] = False
            elif (joy_msg.buttons[joystick_button_mapping["LB"]]):
                self.prev_state[lower_dumper] = self.curr_state[lower_dumper]
                self.curr_state[lower_dumper] = True
            elif (joy_msg.buttons[joystick_button_mapping["RB"]]):
                self.prev_state[raise_dumper] = self.curr_state[raise_dumper]
                self.curr_state[raise_dumper] = True
            else:
                self.prev_state[raise_dumper] = self.curr_state[raise_dumper]
                self.curr_state[raise_dumper] = False
                self.prev_state[lower_dumper] = self.curr_state[lower_dumper]
                self.curr_state[lower_dumper] = False
        

        else: #what happens if auto flag is on (figure out priority, what the manual commands need to be, method to assign value to the publisher)
            
            self.prev_state[exc_routine] = self.curr_state[exc_routine]            
            self.curr_state[exc_routine] = joy_msg.buttons[joystick_button_mapping[excavation_routine_button]]

            self.prev_state[dep_routine] = self.curr_state[dep_routine]        
            self.curr_state[dep_routine] = joy_msg.buttons[joystick_button_mapping[deposit_routine_button]]

            if (self.curr_state[exc_routine] and self.curr_state[dep_routine]): #if both pressed, ignore both
                self.curr_state[exc_routine] = False
                self.curr_state[dep_routine] = False
            




        for i in range(len(self.curr_state)):
            if (self.prev_state[i] != self.curr_state[i]):
                if (i < 2):
                    pub_msg = Float32()
                    pub_msg.data = float(self.curr_state[i])
                else:
                    pub_msg = Bool()
                    pub_msg.data = bool(self.curr_state[i])
                print(pub_msg)
                self.controller_publishers[i].publish(pub_msg)
                                

        

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

    rclpy.shutdown()


if __name__ == '__main__':
    main()



    
#a_button = msg.buttons[0] 
#b_button = msg.buttons[1]
#x_button = msg.buttons[2]
#y_button = msg.buttons[3]
#l_bumper = msg.buttons[4]
#r_bumper = msg.buttons[5]
#back_button = msg.buttons[6]
#start_button = msg.buttons[7]
#home_button = msg.buttons[8]

#left_joy_x = msg.axes[0]
#left_joy_y = msg.axes[1]
#left_trigger = msg.axes[2]
#right_joy_x = msg.axes[3]
#right_joy_y = msg.axes[4]
#right_trigger = msg.axes[5]
#arrow_pad_x = msg.axes[6]
#arrow_pad_y = msg.axes[7]









#     def joy_response(msg, self):
#         print (msg)
