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

        self.linear_actuator = LinearActuator(123, 456, 789)

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
        self.hold_time = 5
        self.motor_speed = 0.5
        self.motor_rest = 0

    def dumper_max_limit_switch_response(self, message: Bool):
        self.max_limit = message.data

    def dumper_min_limit_switch_response(self, message: Bool):
        self.min_limit = message.data

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator.run_motor(self.motor_rest)
            return
        
        if self.dep_routine_val:
            if self.direction and self.max_limit:
                self.direction = False
                time.sleep(self.hold_time)
            if not self.direction and self.min_limit:
                self.should_run = False
                self.dep_routine_val = False

                self.dep_routine_publisher.publish(Bool(data=False))

        if (self.direction and not self.max_limit) or (not self.direction and not self.min_limit):
            self.linear_actuator.set_direction(self.direction)
            self.run_motor(self.motor_speed)
        else:
            self.linear_actuator.run_motor(self.motor_rest)

    def dep_routine_response(self, message: Bool):
        self.dep_routine_val = message.data

        if self.dep_routine_val:
            self.should_run = True
            self.direction = True


    def raise_dumper_response(self, message: Bool):
        if self.dep_routine_val:
            return
        
        if not self.dep_routine_val and message.data:
            self.should_run = True
            self.direction = True

        else:
            self.should_run = False

    def lower_dumper_response(self, message: Bool):
        if not self.dep_routine_val and message.data:
            self.should_run = True
            self.direction = False

        else:
            self.should_run = False
