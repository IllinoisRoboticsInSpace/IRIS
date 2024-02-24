import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist

from sensor_msgs.msg import Joy

from std_msgs.msg import String

import serial

class arduino_comms_node(Node):

    def __init__(self):
        super().__init__('arduino_comms_node')

        # self.subscription = self.create_subscription(
        #     Twist, '/rover/cmd_vel', self.twist_callback, 10) #subscribes to cmd_vel topic

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10) #subscribes to joy topic
        
        self.get_logger().info(f"Created node {self.get_name()}")

        self.publisher_ = self.create_publisher(String, 'controller', 10) #publishes to 'controller' topic

        # USB Cable should be connected to programming port from the jetson
        self.arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 9600, timeout = .1) #initializes arduino serial port. make sure baudrate is same for arduino and python code

        # Motor ID:
        # * 0: Left Drive
        # * 1: Right Drive
        # * 2: Left Back Collection Motor
        # * 3: Right Back Collection Motor
        # * 4: Excavator Internal Motor
        # * 5: Excavator Threaded Rod Actuator
        # * 6: Excavator Pivoting Linear Actuator
        # * 7: stops motors
        self.numMotor = [0,1,2,3,4,5,6]
        self.power = [0,0,0,0,0,0,0]

        #motor states
        self.LEFT_DRIVE = 0
        self.RIGHT_DRIVE = 0
        self.LEFT_BACK_COLL = 0
        self.RIGHT_BACK_COLL = 0
        self.EXC_INTERNAL = 0
        self.EXC_THREAD_ROD = 0
        self.EXC_PIVOT_LIN = 0


    ####################################################################################################

    #IMPORTANT FUNCTIONS:
    def power_to_bytes(self, i) -> bytes:      #converts self.power into bytes
        def largestOneIndex(curr:int):
            curr = int(curr)
            i = 15
            while (i >= 0):
                if (curr & int(1 << i)):
                    break
                i-= 1
            return i
        x = int(abs(self.power[i]) + 256 * self.numMotor[i])
        #x is two bits, power must be between -127 and 127. probably.
        #first byte should be # of motor + CRC
        #second byte is power (most significant bit is negative sign)
        if(self.power[i] < 0):
            x += 128
        CRC = 18
        msg = int(x % 2048)
        checksum = int(x / 2048)
        curr = int(msg*32 + checksum)
        a = 0
        index = 0
        while (curr >= 32):
            index = largestOneIndex(curr)
            a = CRC << (index - 4)
            curr ^= a
        y = x + curr*256*8
        return y.to_bytes((y.bit_length() + 7) // 8, 'big')
    # def sendPower(self):
    #     for i in range(len(self.power)):
    #         self.arduino.write(self.power_to_bytes(i)) #writes power to serial0
    
    ####################################################################################################
    def limitPower(self):
        for p in self.power:
            if(p > 127):
                p = 127
            elif(p< -127):
                p = -127
    def display_power(self, i):
        returnMsg = String()
        returnMsg.data = 'power = ' + str(self.power[i]) + ' bytes: ' + str(self.power_to_bytes(i))
        self.publisher_.publish(returnMsg)

    # def twist_callback(self, twist_msg: Twist):
    #     x = max(min(twist_msg.linear.x, 1.0), -1.0)
    #     z = max(min(twist_msg.angular.z, 1.0), -1.0)
    #     scale = 80
    #     self.power[0] = -int((x - z) * scale) 
    #     self.power[1] = int((x + z) * scale)

    def joy_callback(self, joy_msg: Joy):
        scale = 80      #power scale
        axes_error = 0.5

        #Left Drive
        curr_left_drive_state = (abs(joy_msg.axes[1])>axes_error)
        if curr_left_drive_state != self.LEFT_DRIVE:
            self.LEFT_DRIVE = curr_left_drive_state
            self.power[0] = scale * joy_msg.axes[1] * self.LEFT_DRIVE

            self.limitPower()
            self.display_power(0)
            self.arduino.write(self.power_to_bytes(0))
        
        #Right Drive
        curr_right_drive_state = (abs(joy_msg.axes[4])>axes_error)
        if curr_right_drive_state != self.RIGHT_DRIVE:
            self.RIGHT_DRIVE = curr_right_drive_state
            self.power[1] = scale * joy_msg.axes[4] * self.RIGHT_DRIVE

            self.limitPower()
            self.display_power(1)
            self.arduino.write(self.power_to_bytes(1))
        
        #Left Back Collection
        if joy_msg.buttons[4] != self.LEFT_BACK_COLL:
            self.LEFT_BACK_COLL = joy_msg.buttons[4]
            self.power[2] = scale * self.LEFT_BACK_COLL

            self.limitPower()
            self.display_power(2)
            self.arduino.write(self.power_to_bytes(2))
        
        #Right Back Collection
        if joy_msg.buttons[5] != self.RIGHT_BACK_COLL:
            self.RIGHT_BACK_COLL = joy_msg.buttons[5]
            self.power[3] = scale * self.RIGHT_BACK_COLL

            self.limitPower()
            self.display_power(3)
            self.arduino.write(self.power_to_bytes(3))
        
        #Excavator Internal Motor
        if joy_msg.buttons[0] != self.EXC_INTERNAL:
            self.EXC_INTERNAL = joy_msg.buttons[0]
            self.power[4] = scale * self.EXC_INTERNAL

            self.limitPower()
            self.display_power(4)
            self.arduino.write(self.power_to_bytes(4))
        
        #Excavator Threaded Rod Actuator
        if joy_msg.buttons[1] != self.EXC_THREAD_ROD:
            self.EXC_THREAD_ROD = joy_msg.buttons[1]
            self.power[5] = scale * self.EXC_THREAD_ROD

            self.limitPower()
            self.display_power(5)
            self.arduino.write(self.power_to_bytes(5))
        
        #Excavator Pivoting Linear Actuator
        if joy_msg.buttons[2] != self.EXC_PIVOT_LIN:
            self.EXC_PIVOT_LIN = joy_msg.buttons[2]
            self.power[6] = scale * self.EXC_PIVOT_LIN

            self.limitPower()
            self.display_power(6)
            self.arduino.write(self.power_to_bytes(6))
        
        #Power Off
        if joy_msg.buttons[8]==1:
            for i in range(len(self.power)):
                self.power[i] = 0

                self.display_power(i)
                self.arduino.write(self.power_to_bytes(i))
        
        #self.get_logger().info(f"power:{self.power}")
        # self.sendPower()
        

def main(args=None):
    rclpy.init(args=args)

    contro = arduino_comms_node()
    try:
        rclpy.spin(contro)
    except KeyboardInterrupt:
        pass
    finally:
        contro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
