import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from linear_actuator import LinearActuator


class Excavator(Node):
    def __init__(self):
        super().__init__('Excavator')

        self.linear_actuator = LinearActuator('S')

        self.exc_routine_publisher = self.create_publisher(
            Bool, 'exc_routine', 10)
        self.tread_speed_left_exc_routine_publisher = self.create_publisher(Float32, 'tread_speed_left_exc_routine', 10)
        self.tread_speed_right_exc_routine_publisher = self.create_publisher(Float32, 'tread_speed_right_exc_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)
        self.raise_excavator_teleop_subscriber = self.create_subscription(
            Bool, 'raise_scooper_teleop', self.raise_excavator_response, 10)
        self.lower_excavator_teleop_subscriber = self.create_subscription(
            Bool, 'lower_scooper_teleop', self.lower_excavator_response, 10)
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
        self.hold_start_time = 0

        # self.motor_speed = 0.5
        # self.motor_rest = 0.0

        self.tread_speed = 0.3
        
        # timing based scooping routine
        self.lower_duration = 35.0
        self.drive_duration = 2.0
        self.raise_duration = 35.0

    def excavator_max_limit_switch_response(self, message: Bool):
        self.max_limit = message.data

    def excavator_min_limit_switch_response(self, message: Bool):
        self.min_limit = message.data

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator.run_motor(False)
            return

        
        if (self.forward and not self.max_limit) or (not self.forward and not self.min_limit):
            self.linear_actuator.set_direction(self.forward)
            self.linear_actuator.run_motor(True)
        else:
            self.linear_actuator.run_motor(False)

        #if self.exc_routine_val:
        #    elapsed = (time.time_ns() - self.phase_start_time) / 1e9
        #    self.get_logger().info('Entering excavation routine...')

        #    if self.routine_phase == 0:
        #        # part 1: lower scoop for 15 seconds
        #        self.get_logger().info('Lowering scoop...')
        #        if elapsed >= self.lower_duration:
        #            self.routine_phase = 1
        #            self.phase_start_time = time.time_ns()

        #    elif self.routine_phase == 1:
        #        # part 2: drive forward for 2 seconds
        #        self.get_logger().info(f'Waiting for 2 seconds...')
                #self.tread_speed_left_exc_routine_publisher.publish(Float32(data=self.tread_speed * 2))
                #self.tread_speed_right_exc_routine_publisher.publish(Float32(data=self.tread_speed * 2))
                
        #        if elapsed >= self.drive_duration:
        #            self.routine_phase = 2
        #            self.phase_start_time = time.time_ns()
        #            self.linear_actuator.set_direction(True)
        #            self.linear_actuator.run_motor(True)

        #    elif self.routine_phase == 2:
        #        # part 3: raise scoop for 15 seconds
        #        self.get_logger().info('Raising scoop...')
        #        #self.linear_actuator.set_direction(True)
        #        #self.linear_actuator.run_motor(True)
        #        if elapsed >= self.raise_duration:
        #            self.routine_phase = 3

        #    else:
        #        # routine complete
        #        self.get_logger().info('Routine complete...')
        #        self.linear_actuator.run_motor(False)
        #        self.should_run = False
        #        self.exc_routine_val = False
        #        self.exc_routine_publisher.publish(Bool(data=False))

        #else:
        #    if (self.forward and not self.max_limit) or (not self.forward and not self.min_limit):
        #        self.linear_actuator.set_direction(self.forward)
        #        self.linear_actuator.run_motor(True)
        #    else:
        #        self.linear_actuator.run_motor(False)
        #if self.exc_routine_val:
        #    if self.forward and self.max_limit:
        #        if not self.hold_initiated:
        #            self.hold_initiated = True
        #            self.hold_start_time = time.time_ns()
        #            self.tread_speed_left_exc_routine_publisher.publish(Float32(self.tread_speed))
        #            self.tread_speed_right_exc_routine_publisher.publish(Float32(self.tread_speed))

         #       if (self.hold_start_time + self.hold_time < time.time_ns()):                    
         #           self.forward = False
         #           self.hold_initiated = False
         #           self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
         #           self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))                

                

          #  if not self.forward and self.min_limit:
         #       self.forward = True
         #       self.should_run = False
         #       self.exc_routine_val = False

         #       self.exc_routine_publisher.publish(Bool(data=False))


    def exc_routine_response(self, message: Bool):
        self.exc_routine_val = False
        #self.exc_routine_val = message.data
        #if message.data and not self.should_run:
        #    self.get_logger().info('Triggering excavation routine...')
        #    self.should_run = True
        #    self.routine_phase = 0
        #    self.phase_start_time = time.time_ns()
        #    self.linear_actuator.set_direction(False)
        #    self.linear_actuator.run_motor(True)

        #else:
        #    self.get_logger().info('Disabling excavation routine...')

    def raise_excavator_response(self, message: Bool):
        self.get_logger().info("Raising excavator")
        if self.exc_routine_val:
            return

        if message.data:
            self.should_run = True
            self.forward = True
        else:
            self.should_run = False

    def lower_excavator_response(self, message: Bool):
        self.get_logger().info("Lowering excavator")
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
