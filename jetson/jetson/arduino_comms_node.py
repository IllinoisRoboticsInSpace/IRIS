import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool, Float32

from sensor_msgs.msg import Joy

from std_msgs.msg import String

import serial
from teleop import get_gamepad_mapping

from motor_driver import MotorDriver

N_INPUTS = 9

class arduino_comms_node(Node):

    def __init__(self):
        super().__init__('arduino_comms_node')

        self.motor_driver = MotorDriver(debugMode=True)

        self.gamepad_subscribers = [None] * N_INPUTS

        # self.mapping = {"left drive": LEFT_DRIVE, "right drive": RIGHT_DRIVE, "left back cltn motor": LEFT_BACK_COLL, 
        #                         "right back cltn motor": RIGHT_BACK_COLL, "exc internal motor": EXC_INTERNAL, "exc thread rod act": EXC_THREAD_ROD,
        #                         "exc pivot lin act": EXC_PIVOT_LIN, "auto mode": AUTO_MODE, "stop mode": STOP_MODE}
        self.gamepad_mapping = get_gamepad_mapping()

        self.gamepad_subscribers[self.gamepad_mapping["left drive"]] = self.create_publisher(Float32, '/gamepad/left_drive', self.motor_driver_callback, 10)
        self.gamepad_subscribers[self.gamepad_mapping["right drive"]] = self.create_publisher(Float32, '/gamepad/right_drive', self.motor_driver_callback, 10)
        self.gamepad_subscribers[self.gamepad_mapping["left back cltn motor"]] = self.create_publisher(Bool, '/gamepad/left_back_coll', self.motor_driver_callback, 10)
        self.gamepad_subscribers[self.gamepad_mapping["right back cltn motor"]] = self.create_publisher(Bool, '/gamepad/right_back_coll', self.motor_driver_callback, 10)
        self.gamepad_subscribers[self.gamepad_mapping["exc internal motor"]] = self.create_publisher(Bool, '/gamepad/exc_internal', self.motor_driver_callback, 10)    
        self.gamepad_subscribers[self.gamepad_mapping["exc thread rod act"]] = self.create_publisher(Bool, '/gamepad/exc_thread_rod', self.motor_driver_callback, 10)
        self.gamepad_subscribers[self.gamepad_mapping["exc pivot lin act"]] = self.create_publisher(Bool, '/gamepad/exc_pivot_lin', self.motor_driver_callback, 10)

        self.gamepad_subscribers[self.gamepad_mapping["auto mode"]] = self.create_publisher(Bool, '/gamepad/auto_mode', self.motor_driver_callback, 10)
        self.gamepad_subscribers[self.gamepad_mapping["stop mode"]] = self.create_publisher(Bool, '/gamepad/stop_mode', self.motor_driver_callback, 10)
        
        self.get_logger().info(f"Created node {self.get_name()}")

        # USB Cable should be connected to programming port from the jetson
        # initializes arduino serial port. make sure baudrate is same for arduino and python code
        
        self.arduino = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600, timeout = .1)

    def motor_driver_callback(self, msg):
        self.get_logger().info("Data received: %s" % msg.data)

        if()


def main(args=None):
    rclpy.init(args=args)

    arduino_com_subscriber = arduino_comms_node()
    try:
        rclpy.spin(arduino_com_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        arduino_com_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
