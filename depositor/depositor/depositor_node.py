
import rclpy
from rclpy.qos import QoSProfile, LivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration
import copy
from dataclasses import dataclass
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time

from linear_actuator import LinearActuator

PHASE_INIT = 0
PHASE_RAISE = 1
PHASE_HOLD = 2
PHASE_LOWER = 3

class Depositer(Node):
    def __init__(self):
        super().__init__('Depositer')

        self.linear_actuator = LinearActuator('D')

        self.dep_routine_publisher = self.create_publisher(
            Bool, 'dep_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)

        self.raise_dumper_teleop_subscriber = self.create_subscription(
            Bool, 'raise_dumper_teleop', self.raise_dumper_response, 10)
        self.lower_dumper_teleop_subscriber = self.create_subscription(
            Bool, 'lower_dumper_teleop', self.lower_dumper_response, 10)
        self.dep_routine_subscription = self.create_subscription(
            Bool, 'dep_routine', self.dep_routine_response, 10)

        self.dep_routine_val = False

        self.should_run = False
        self.up = False

        self.lower_duration = 40.0 * 1e9
        self.hold_duration = 10.0 * 1e9
        self.raise_duration = 35.0 * 1e9

        self.routine_phase = PHASE_INIT
        self.routine_phase_start_time = 0

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator.run_motor(False)
        else:        
            self.linear_actuator.set_direction(self.up)
            self.linear_actuator.run_motor(True)

        if (self.routine_phase == PHASE_INIT):
            pass
        elif (self.routine_phase == PHASE_RAISE):
            if (time.time_ns() - self.routine_phase_start_time > self.raise_duration):
                self.routine_phase = PHASE_HOLD
                self.routine_phase_start_time = time.time_ns()
                self.up = False
                self.should_run = False
        elif (self.routine_phase == PHASE_HOLD):
            if (time.time_ns() - self.routine_phase_start_time > self.hold_duration):
                self.routine_phase = PHASE_LOWER
                self.routine_phase_start_time = time.time_ns()
                self.up = False
                self.should_run = True
        elif (self.routine_phase == PHASE_LOWER):
            if (time.time_ns() - self.routine_phase_start_time > self.lower_duration):
                self.routine_phase = PHASE_INIT
                self.dep_routine_val = False
                self.dep_routine_publisher.publish(Bool(data=False))
                self.up = True
                self.should_run = False
                


    def dep_routine_response(self, message: Bool):
        self.dep_routine_val = message.data
        if message.data and not self.should_run:
           self.get_logger().info('Triggering dumping routine...')

           self.routine_phase = PHASE_RAISE
           self.routine_phase_start_time = time.time_ns()

           self.up = True
           self.should_run = True

    def raise_dumper_response(self, message: Bool):
        self.get_logger().info("Raising dumper")
        if self.dep_routine_val:
            self.routine_phase = PHASE_INIT
            self.dep_routine_val = False
            self.dep_routine_publisher.publish(Bool(data=False))
        
        if message.data:
            self.should_run = True
            self.up = True

        else:
            self.should_run = False

    def lower_dumper_response(self, message: Bool):
        self.get_logger().info("Lowering dumper")
        if self.dep_routine_val:
            self.routine_phase = PHASE_INIT
            self.dep_routine_val = False
            self.dep_routine_publisher.publish(Bool(data=False))
        
        if message.data:
            self.should_run = True
            self.up = False

        else:
            self.should_run = False


    def destroy_node(self):
        self.linear_actuator.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    dep = Depositer()
    try:
        rclpy.spin(dep)
    except KeyboardInterrupt:
        pass
    finally:
        dep.destroy_node()
        rclpy.shutdown()
