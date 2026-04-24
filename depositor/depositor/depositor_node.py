
import rclpy
import copy
from dataclasses import dataclass
from rclpy.node import Node
from std_msgs.msg import Bool
import time

from linear_actuator import LinearActuator

class Depositer(Node):
    def __init__(self):
        super().__init__('Depositer')

        self.linear_actuator = [LinearActuator("/dev/gpiochip0", 53, 113), LinearActuator("/dev/gpiochip0", 124, 52)]

        self.dep_routine_publisher = self.create_publisher(
            Bool, 'dep_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)

        self.raise_dumper_teleop_subscriber = self.create_subcription(
            Bool, 'raise_dumper_teleop', self.raise_dumper_response, 10)
        self.lower_dumper_teleop_subscriber = self.create_subcription(
            Bool, 'lower_dumper_teleop', self.lower_dumper_response, 10)
        self.dep_routine_subscription = self.create_subscription(
            Bool, 'dep_routine', self.dep_routine_response, 10)
        self.dumper_max_limit_switch_subscription = self.create_subscription(
            Bool, 'dumper_max_limit_switch', self.dumper_max_limit_switch_response, 10)
        self.dumper_min_limit_switch_subscription = self.create_subscription(
            Bool, 'dumper_min_limit_switch', self.dumper_min_limit_switch_response, 10)

        self.dep_routine_val = False
        self.max_limit = False
        self.min_limit = False
        self.should_run = False
        self.direction = False
        self.hold_time = 5e9  # 5 ns
        self.hold_initiated = False
        self.hold_start_time = 0

    def dumper_max_limit_switch_response(self, message: Bool):
        self.max_limit = message.data

    def dumper_min_limit_switch_response(self, message: Bool):
        self.min_limit = message.data

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator[0].run_motor(False)
            self.linear_actuator[1].run_motor(False)
            return
        
        if self.dep_routine_val:
            if self.direction and self.max_limit:
                if not self.hold_initiated:
                    self.hold_initiated = True
                    self.hold_start_time = time.time_ns()
                
                if (self.hold_start_time + self.hold_time < time.time_ns()):
                    self.direction = False
                    self.hold_initiated = False

            if not self.direction and self.min_limit:
                self.should_run = False
                self.dep_routine_val = False

                self.dep_routine_publisher.publish(Bool(data=False))

        if (self.direction and not self.max_limit) or (not self.direction and not self.min_limit):
            self.linear_actuator[0].set_direction(self.direction)
            self.linear_actuator[1].set_direction(self.direction)
            self.linear_actuator[0].run_motor(True)
            self.linear_actuator[1].run_motor(True)
        else:
            self.linear_actuator[0].run_motor(False)
            self.linear_actuator[1].run_motor(False)

    def dep_routine_response(self, message: Bool):
        self.dep_routine_val = message.data

        if message.data:
            self.should_run = True
            self.direction = True


    def raise_dumper_response(self, message: Bool):
        if self.dep_routine_val:
            return
        
        if message.data:
            self.should_run = True
            self.direction = True

        else:
            self.should_run = False

    def lower_dumper_response(self, message: Bool):
        if self.dep_routine_val:
            return
        
        if message.data:
            self.should_run = True
            self.direction = False

        else:
            self.should_run = False


    def destroy_node(self):
        self.linear_actuator[0].release()
        self.linear_actuator[1].release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    exc = Excavator()
    try:
        rclpy.spin(exc)
    except KeyboardInterrupt:
        pass
    finally:
        exc.destroy_node()
        rclpy.shutdown()