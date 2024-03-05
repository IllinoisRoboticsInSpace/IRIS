import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool, Float32

from sensor_msgs.msg import Joy

from std_msgs.msg import String

import serial
from teleop import gamepad_node

class arduino_comms_node(Node):

    def __init__(self):
        super().__init__('arduino_comms_node')

        self.gamepad_subscribers = [None] * 9

        self.gamepad_subscribers[gamepad_node.LEFT_DRIVE] = self.create_publisher(Float32, '/gamepad/left_drive', 10)
        self.gamepad_subscribers[gamepad_node.RIGHT_DRIVE] = self.create_publisher(Float32, '/gamepad/right_drive', 10)
        self.gamepad_subscribers[gamepad_node.LEFT_BACK_COLL] = self.create_publisher(Bool, '/gamepad/left_back_coll', 10)
        self.gamepad_subscribers[gamepad_node.RIGHT_BACK_COLL] = self.create_publisher(Bool, '/gamepad/right_back_coll', 10)
        self.gamepad_subscribers[gamepad_node.EXC_INTERNAL] = self.create_publisher(Bool, '/gamepad/exc_internal', 10)    
        self.gamepad_subscribers[gamepad_node.EXC_THREAD_ROD] = self.create_publisher(Bool, '/gamepad/exc_thread_rod', 10)
        self.gamepad_subscribers[gamepad_node.EXC_PIVOT_LIN] = self.create_publisher(Bool, '/gamepad/exc_pivot_lin', 10)

        self.gamepad_subscribers[gamepad_node.AUTO_MODE] = self.create_publisher(Bool, '/gamepad/auto_mode', 10)
        self.gamepad_subscribers[gamepad_node.STOP_MODE] = self.create_publisher(Bool, '/gamepad/stop_mode', 10)
        
        self.get_logger().info(f"Created node {self.get_name()}")

        # self.publisher = self.create_publisher(String, 'controller', 10) #publishes to 'controller' topic

        # USB Cable should be connected to programming port from the jetson
        # initializes arduino serial port. make sure baudrate is same for arduino and python code
        
        self.arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 9600, timeout = .1)

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
