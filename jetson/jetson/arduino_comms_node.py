import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool, Float32

from sensor_msgs.msg import Joy

from std_msgs.msg import String

import serial
from teleop import get_gamepad_mapping

from motor_driver import MotorDriver

from functools import partial
from threading import Lock

N_INPUTS = 9
FULL_MTR_PWR = 0.8 # so we dont break things

class arduino_comms_node(Node):

    def __init__(self):
        super().__init__('arduino_comms_node')

        self.lock = Lock()
        self.motor_driver = MotorDriver(debugMode=True)

        self.gamepad_subscribers = [None] * N_INPUTS

        self.gamepad_mapping = get_gamepad_mapping()

        for key, value in self.gamepad_mapping.items():
            if key in ["left_drive", "right_drive"]:
                msg_type = Float32
            else:
                msg_type = Bool

            callback = partial(self.motor_driver_callback, motor=key)
            self.gamepad_subscribers[value] = self.create_subscription(msg_type, f"/gamepad/{key}", callback, 10)

        self.get_logger().info(f"Created node {self.get_name()}")

        # USB Cable should be connected to programming port from the jetson
        # initializes arduino serial port. make sure baudrate is same for arduino and python code
        
        self.arduino = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600, timeout = .1)


    # mapping = {"left_drive": LEFT_DRIVE, "right_drive": RIGHT_DRIVE, "left_back_cltn_mtr": LEFT_BACK_COLL, 
    #                             "right_back_cltn_mtr": RIGHT_BACK_COLL, "exc_intrnl_mtr": EXC_INTERNAL, "exc_thrd_rod_actr": EXC_THREAD_ROD,
    #                             "exc_pvt_lin_actr": EXC_PIVOT_LIN, "auto_mode": AUTO_MODE, "stop_mode": STOP_MODE}
    
    def motor_driver_callback(self, motor, msg):
        with self.lock:
            self.get_logger().info(f"Data received on {motor}: %s" % msg.data)

            if motor == "left_drive":
                self.motor_driver.turnMotor(self.gamepad_mapping[motor], msg.data)

            elif motor == "right_drive":
                self.motor_driver.turnMotor(self.gamepad_mapping[motor], msg.data)

            elif motor == "left_back_cltn_mtr":
                pass

            elif motor == "right_back_cltn_mtr":
                pass

            elif motor == "exc_intrnl_mtr":
                pass

            elif motor == "exc_thrd_rod_actr":
                # self.motor_driver.turnMotor(self.gamepad_mapping[motor], FULL_MTR_PWR)
                pass

            elif motor == "exc_pvt_lin_actr":
                # self.motor_driver.turnMotor(self.gamepad_mapping[motor], FULL_MTR_PWR)
                pass

            elif motor == "auto_mode":
                self.switch_to_auto_mode()

            elif motor == "stop_mode":
                self.motor_driver.stopMotors()

    def switch_to_auto_mode(self):
        # TODO: update subscription to nav2 nodes
        for key, value in self.gamepad_mapping.items():
            if key in ["left_drive", "right_drive"]:
                msg_type = Float32
            else:
                msg_type = Bool

            callback = partial(self.motor_driver_callback, motor=key)
            self.gamepad_subscribers[value] = self.create_subscription(msg_type, f"/gamepad/{key}", callback, 10)
            
        


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
