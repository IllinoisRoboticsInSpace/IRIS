
import rclpy
import copy
from dataclasses import dataclass
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time

from linear_actuator import LinearActuator

class Depositer(Node):
    def __init__(self):
        super().__init__('Depositer')

        self.linear_actuator = LinearActuator('D')

        self.dep_routine_publisher = self.create_publisher(
            Bool, 'dep_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)

        self.tread_speed_left_dep_routine_publisher = self.create_publisher(Float32, 'tread_speed_left_teleop', 10)
        self.tread_speed_right_dep_routine_publisher = self.create_publisher(Float32, 'tread_speed_right_teleop', 10)

        self.raise_dumper_teleop_subscriber = self.create_subscription(
            Bool, 'raise_dumper_teleop', self.raise_dumper_response, 10)
        self.lower_dumper_teleop_subscriber = self.create_subscription(
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

        self.get_logger().info("Initialized pubs and subs")

        self.lower_duration = 10.0
        self.drive_duration = 30.0
        self.raise_duration = 10.0

        self.tread_speed = 0.2

    def dumper_max_limit_switch_response(self, message: Bool):
        self.max_limit = message.data

    def dumper_min_limit_switch_response(self, message: Bool):
        self.min_limit = message.data

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator.run_motor(False)
            return
        
        if (self.direction and not self.max_limit) or (not self.direction and not self.min_limit):
            self.linear_actuator.set_direction(self.direction)
            self.linear_actuator.run_motor(True)
        else:
            self.linear_actuator.run_motor(False)
        
        #if self.dep_routine_val:
        #    elapsed = (time.time_ns() - self.phase_start_time) / 1e9
        #    self.get_logger().info(f'Scooping routine, elapsed = {elapsed}...')

        #    if self.routine_phase == 0:
        #        # part 1: lower scoop for 15 seconds
        #        self.get_logger().info('Lowering scoop...')
        #        if elapsed >= self.lower_duration:
        #            self.routine_phase = 3
            
        #    elif self.routine_phase == 1:
        #        # part 2: drive for 2 seconds
        #        self.get_logger().info('Driving for 2 seconds...')
        #        self.tread_speed_left_dep_routine_publisher.publish(Float32(data=self.tread_speed * 2))
        #        self.tread_speed_right_dep_routine_publisher.publish(Float32(data=self.tread_speed * 2))

        #        if elapsed >= self.drive_duration:
        #            self.routine_phase = 0
        #            self.phase_start_time = time.time_ns()
        #            self.linear_actuator.set_direction(False)
        #            self.linear_actuator.run_motor(True)

        #    elif self.routine_phase == 2:
        #        # part 3: raise scoop for 15 seconds
        #        self.get_logger().info('Raising scoop...')
        #        #self.linear_actuator.set_direction(True)
        #        #self.linear_actuator.run_motor(True)
        #        if elapsed >= self.raise_duration:
        #            self.routine_phase = 1
        #            self.phase_start_time = time.time_ns()
        #            self.linear_actuator.run_motor(False)

        #    else:
        #        # routine complete
        #        self.get_logger().info('Routine complete...')
        #        self.linear_actuator.run_motor(False)
        #        self.should_run = False
        #        self.dep_routine_val = False
        #        self.dep_routine_publisher.publish(Bool(data=False))

        #else:
        #    if (self.direction and not self.max_limit) or (not self.direction and not self.min_limit):
        #        self.linear_actuator.set_direction(self.direction)
        #        self.linear_actuator.run_motor(True)
        #    else:
        #        self.linear_actuator.run_motor(False)

#        if self.dep_routine_val:
#            if self.direction and self.max_limit:
#                if not self.hold_initiated:
#                    self.hold_initiated = True
#                    self.hold_start_time = time.time_ns()
#                
#                if (self.hold_start_time + self.hold_time < time.time_ns()):
#                    self.direction = False
#                    self.hold_initiated = False
#
#            if not self.direction and self.min_limit:
#                self.should_run = False
#                self.dep_routine_val = False
#
#                self.dep_routine_publisher.publish(Bool(data=False))


    def dep_routine_response(self, message: Bool):
        self.dep_routine_val = False
        #self.dep_routine_val = message.data
        #if message.data and not self.should_run:
        #    self.get_logger().info('Triggering scooping routine...')
        #    self.should_run = True
        #    self.routine_phase = 2
        #    self.phase_start_time = time.time_ns()
        #    self.linear_actuator.set_direction(True)
        #    self.linear_actuator.run_motor(True)

        #else:
        #    self.get_logger().info('Disabling scooping routine...')

    def raise_dumper_response(self, message: Bool):
        self.get_logger().info("Raising dumper")
        if self.dep_routine_val:
            return
        
        if message.data:
            self.should_run = True
            self.direction = True

        else:
            self.should_run = False

    def lower_dumper_response(self, message: Bool):
        self.get_logger().info("Lowering dumper")
        if self.dep_routine_val:
            return
        
        if message.data:
            self.should_run = True
            self.direction = False

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
