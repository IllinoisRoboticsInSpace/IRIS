import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from linear_actuator import LinearActuator


class Excavator(Node):
    def __init__(self):
        super().__init__('Excavator')

        self.linear_actuator = LinearActuator(123, 456, 789)

        self.exc_routine_publisher = self.create_publisher(
            Bool, 'exc_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)
        self.raise_excavator_teleop_subscriber = self.create_subcription(
            Bool, 'raise_excavator_teleop', self.raise_excavator_response, 10)
        self.lower_excavator_teleop_subscriber = self.create_subcription(
            Bool, 'lower_excavator_teleop', self.lower_excavator_response, 10)
        self.exc_routine_subscription = self.create_subscription(
            Bool, 'exc_routine', self.exc_routine_response, 10)
        self.excavator_max_limit_switch_subscription = self.create_subscription(
            Bool, 'excavator_max_limit_switch', self.excavator_max_limit_switch_response, 10)
        self.excavator_min_limit_switch_subscription = self.create_subscription(
            Bool, 'excavator_min_limit_switch', self.excavator_min_limit_switch_response, 10)

        self.exc_routine_val = False
        self.max_limit = False
        self.min_limit = False
        self.should_run = False
        self.forward = False

        self.motor_speed = 0.5

    def excavator_max_limit_switch_response(self, message: Bool):
        self.max_limit = message.data

    def excavator_min_limit_switch_response(self, message: Bool):
        self.min_limit = message.data

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator.run_motor(0.0)
            return

        if self.exc_routine_val:
            if self.forward and self.max_limit:
                self.forward = False
            if not self.forward and self.min_limit:
                self.forward = True
                self.should_run = False
                self.exc_routine_val = False

                self.exc_routine_publisher.publish(Bool(data=False))

        if (self.forward and not self.max_limit) or (not self.forward and not self.min_limit):
            self.linear_actuator.set_direction(self.forward)
            self.run_motor(self.motor_speed)
        else:
            self.linear_actuator.run_motor(0.0)

    def exc_routine_response(self, message: Bool):
        self.exc_routine_val = message.data
        if message.data:
            self.should_run = True
            self.forward = True

    def raise_excavator_response(self, message: Bool):
        if self.exc_routine_val:
            return

        if message.data:
            self.should_run = True
            self.forward = True
        else:
            self.should_run = False

    def lower_excavator_response(self, message: Bool):
        if self.exc_routine_val:
            return

        if message.data:
            self.should_run = True
            self.forward = False
        else:
            self.should_run = False

    def destroy_node(self):
        self.linear_actuator.release()
        super().destroy_node()
