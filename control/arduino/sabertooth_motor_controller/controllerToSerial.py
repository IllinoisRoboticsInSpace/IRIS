import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy

import serial

class controllerToSerial(Node):

    def __init__(self):
        super().__init__('controllerToSerial')

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10) #subscribes to Joy topic

        self.publisher_ = self.create_publisher(String, 'controller', 10) #publishes to 'controller' topic

        self.arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 9600, timeout = .1) #initializes arduino serial port. make sure baudrate is same for arduino and python code

        self.working = False
        self.button0_prev = False
        self.numMotor = [1,2]
        self.power = [0,0]

    ####################################################################################################

    #IMPORTANT FUNCTIONS:
    def power_to_bytes(self, i) -> bytes:      #converts self.power into bytes
        def largestOneIndex(curr:int):
            i = 15
            while (i >= 0):
                if (curr & (1 << i)):
                    break
                i-=1
            return i
        x = abs(self.power[i]) + 256 * self.numMotor[i]
        #x is two bits, power must be between -127 and 127. probably.
        #first byte should be # of motor + CRC
        #second byte is power (most significant bit is negative sign)
        if(self.power[i] < 0):
            x += 128
        CRC = 18
        msg = x % 2048
        checksum = int(x / 2048)
        curr = msg*32 + checksum
        a = 0
        index = 0
        while (curr >= 32):
            index = largestOneIndex(curr)
            a = CRC << (index - 4)
            curr ^= a
        y = x + curr*256*8
        return y.to_bytes((y.bit_length() + 7) // 8, 'big')
    def sendPower(self, i):
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
        returnMsg.data = 'power = ' + str(self.power) + ', bytes:' + self.power_to_bytes(i).hex()
        self.publisher_.publish(returnMsg)

    def joystick_callback(self, msg: Joy):
        if(msg.buttons[0] and not self.button0_prev):   # 'A' switches between functioning and non-functiong
            if(self.working):
                self.working = False
                returnMsg = String()
                returnMsg.data = 'Not Currently Working'
                self.publisher_.publish(returnMsg)
            else:
                self.working = True
                returnMsg = String()
                returnMsg.data = 'Working'
                self.publisher_.publish(returnMsg)
        self.button0_prev = msg.buttons[0]              #makes sure above code runs only once

        if(self.working and msg.axes[1]!=0): 

            self.power[0] = int(msg.axes[1] * 60)  #Set the power to left-joystick y-axis * 60 
                                                #(joystick value is in range [-1,1])
            if(msg.buttons[2]):                 
                self.power[0] *= 2                 # if 'X' is pressed, multiply power by 2 (for fun not necessary)
            
            self.limitPower()
            self.display_power(0)

            self.sendPower(0)
        elif(msg.axes[1]==0 and self.power[0]!=0): #If left joystick y-axis = 0, and the power isn't already 0, set it to 0.
            self.power[0] = 0
            self.display_power(0)
            self.sendPower(0)
        if(self.working and msg.axes[4]!=0): 

            self.power[1] = int(msg.axes[4] * 60)  #Set the power to right-joystick y-axis * 60 
                                                #(joystick value is in range [-1,1])
            if(msg.buttons[2]):                 
                self.power[1] *= 2                 # if 'X' is pressed, multiply power by 2 (for fun not necessary)
            
            self.limitPower()
            self.display_power(1)

            self.sendPower(1)
        elif(msg.axes[4]==0 and self.power[1]!=0): #If right joystick y-axis = 0, and the power isn't already 0, set it to 0.
            self.power[1] = 0
            self.display_power(1)
            self.sendPower(1)
        
        if(msg.buttons[1]):     #If 'B' is pressed, publish the current value of the arduino Serial port
            returnMsg = String()
            returnMsg.data = str(self.arduino.readline())   
            self.publisher_.publish(returnMsg)

def main(args=None):
    rclpy.init(args=args)

    contro = controllerToSerial()
    try:
        rclpy.spin(contro)
    except KeyboardInterrupt:
        pass
    finally:
        contro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()