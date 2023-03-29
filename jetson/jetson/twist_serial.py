import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist

from sensor_msgs.msg import Joy

from std_msgs.msg import String

import serial

class twist_serial(Node):

    def __init__(self):
        super().__init__('twist_serial')

        self.subscription = self.create_subscription(
            Twist, '/rover/cmd_vel', self.twist_callback, 10) #subscribes to cmd_vel topic

        self.subscription = self.create_subscription(
            Joy, '/joy', self.twist_callback, 10) #subscribes to joy topic
        
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
    def sendPower(self):
        for i in range(len(self.power)):
            self.arduino.write(self.power_to_bytes(i)) #writes power to serial0
    
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

    def twist_callback(self, twist_msg: Twist, joy_msg: Joy):
        x = max(min(twist_msg.linear.x, 1.0), -1.0)
        z = max(min(twist_msg.angular.z, 1.0), -1.0)
        scale = 80
        self.power[0] = -int((x - z) * scale) 
        self.power[1] = int((x + z) * scale)
        #added for joystick buttons
        # for button_idx in range(len(joy_msg.buttons)):
        #     self.power[button_idx+2] = 1 #offset to keet power for twist?; placeholder
        if joy_msg.axes[0]=0.0 and round(joy_msg.axes[1])=1.0:
            self.power[0] = scale
        if joy_msg.axes[3]=0.0 and round(joy_msg.axes[4])=1.0:
            self.power[1] = scale
        if joy_msg.buttons[4]=1:
            self.power[2] = scale
        if joy_msg.buttons[5]=1:
            self.power[3] = scale
        if joy_msg.buttons[0]=1:
            self.power[4] = scale
        if joy_msg.buttons[1]=1:
            self.power[5] = scale
        if joy_msg.buttons[2]=1:
            self.power[6] = scale
        # if joy_msg.buttons[8]=1: #power


        self.limitPower()
        self.display_power(0)
        #self.get_logger().info(f"power:{self.power}")
        self.sendPower()
        

def main(args=None):
    rclpy.init(args=args)

    contro = twist_serial()
    try:
        rclpy.spin(contro)
    except KeyboardInterrupt:
        pass
    finally:
        contro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
