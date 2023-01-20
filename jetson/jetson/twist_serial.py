import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist

from std_msgs.msg import String

import serial

class twist_serial(Node):

    def __init__(self):
        super().__init__('twist_serial')

        self.subscription = self.create_subscription(
            Twist, '/rover/cmd_vel', self.twist_callback, 10) #subscribes to cmd_vel topic
        self.get_logger().info(f"Created node {self.get_name()}")

        self.publisher_ = self.create_publisher(String, 'controller', 10) #publishes to 'controller' topic

        self.arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 9600, timeout = .1) #initializes arduino serial port. make sure baudrate is same for arduino and python code

        self.numMotor = [1,2]
        self.power = [0,0]

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

    def twist_callback(self, msg: Twist):
        x = max(min(msg.linear.x, 1.0), -1.0)
        z = max(min(msg.angular.z, 1.0), -1.0)
        scale = 80
        self.power[0] = -int((x - z) * scale) 
        self.power[1] = int((x + z) * scale)
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