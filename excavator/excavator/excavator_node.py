import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from linear_actuator import LinearActuator


class Excavator(Node):
    def __init__(self):
        super().__init__('Excavator')

        self.linear_actuator = [LinearActuator(123, 456, 789), LinearActuator(123, 456, 789)]

        self.exc_routine_publisher = self.create_publisher(
            Bool, 'exc_routine', 10)
        self.tread_speed_left_exc_routine_publisher = self.create_publisher(Float32, 'tread_speed_left_exc_routine', 10)
        self.tread_speed_right_exc_routine_publisher = self.create_publisher(Float32, 'tread_speed_right_exc_routine', 10)

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
        
        self.hold_time = 5e9 # 5ns
        self.hold_initiated = False
        self.holdd_time_start = 0

        # self.motor_speed = 0.5
        # self.motor_rest = 0.0

        self.tread_speed = 0.3
        

    def excavator_max_limit_switch_response(self, message: Bool):
        self.max_limit = message.data

    def excavator_min_limit_switch_response(self, message: Bool):
        self.min_limit = message.data

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator[0].run_motor(False)
            self.linear_actuator[1].run_motor(False)
            return

        if self.exc_routine_val:
            if self.forward and self.max_limit:
                if not self.hold_initiated:
                    self.hold_initiated = True
                    self.hold_start_time = time.time_ns()
                    self.tread_speed_left_exc_routine_publisher.publish(Float32(self.tread_speed))
                    self.tread_speed_right_exc_routine_publisher.publish(Float32(self.tread_speed))

                if (self.hold_start_time + self.hold_time < time.time_ns()):                    
                    self.forward = False
                    self.hold_initiated = False
                    self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                    self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))                


            if not self.forward and self.min_limit:
                self.forward = True
                self.should_run = False
                self.exc_routine_val = False

                self.exc_routine_publisher.publish(Bool(data=False))

        if (self.forward and not self.max_limit) or (not self.forward and not self.min_limit):
            self.linear_actuator[0].set_direction(self.forward)
            self.linear_actuator[1].set_direction(self.forward)
            self.linear_actuator[0].run_motor(True)
            self.linear_actuator[1].run_motor(True)
        else:
            self.linear_actuator[0].run_motor(False)
            self.linear_actuator[1].run_motor(False)

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