import rclpy
from rclpy.qos import QoSProfile, LivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, Duration
import time
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from linear_actuator import LinearActuator

PHASE_INIT = 0
PHASE_LOWER = 1
PHASE_DIG = 2
PHASE_RAISE = 3
PHASE_STORE = 4

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

        self.exc_routine_val = False

        self.should_run = False
        self.up = False

        self.tread_speed = 0.3
        
        # timing based scooping routine
        self.lower_duration = 12.0 * 1e9
        self.dig_duration = 3.0 * 1e9
        self.raise_duration = 12.0 * 1e9
        self.store_duration = 2.0 * 1e9


        self.routine_phase = PHASE_INIT
        self.routine_phase_start_time = 0

    def timer_response(self):
        if not self.should_run:
            self.linear_actuator.run_motor(False)
        else:
            self.linear_actuator.set_direction(self.up)
            self.linear_actuator.run_motor(True)

        if (self.routine_phase == PHASE_INIT):
            self.tread_speed = 0.3
            pass
        elif (self.routine_phase == PHASE_LOWER):
            if (time.time_ns() - self.routine_phase_start_time > self.lower_duration):
                self.routine_phase = PHASE_DIG
                self.routine_phase_start_time = time.time_ns()
                self.up = True
                self.should_run = False

                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=self.tread_speed))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=self.tread_speed))
        elif (self.routine_phase == PHASE_DIG):
            if (time.time_ns() - self.routine_phase_start_time > self.dig_duration):
                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))

                self.routine_phase = PHASE_RAISE
                self.routine_phase_start_time = time.time_ns()
                self.up = True
                self.should_run = True
        elif (self.routine_phase == PHASE_RAISE):
            if (time.time_ns() - self.routine_phase_start_time > self.lower_duration):
                self.routine_phase = PHASE_STORE
                self.routine_phase_start_time = time.time_ns()
                self.up = False
                self.should_run = False
        elif (self.routine_phase == PHASE_STORE):
            if ((time.time_ns() - self.routine_phase_start_time) / 0.5e9) % 2 == 1:
                self.tread_speed = -self.tread_speed
                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=self.tread_speed))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=self.tread_speed))
            if (time.time_ns() - self.routine_phase_start_time > self.store_duration):
                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))
                
                self.routine_phase = PHASE_INIT
                self.exc_routine_val = False
                self.exc_routine_publisher.publish(Bool(data=False))
                self.up = False
                self.should_run = False
        
            


    def exc_routine_response(self, message: Bool):
        self.exc_routine_val = message.data
        if message.data and not self.should_run:
           self.get_logger().info('Triggering scooping routine...')

           self.routine_phase = PHASE_LOWER
           self.routine_phase_start_time = time.time_ns()

           self.up = False
           self.should_run = True

    def raise_excavator_response(self, message: Bool):
        self.get_logger().info("Raising excavator")
        if self.exc_routine_val:
            return

        if message.data:
            self.should_run = True
            self.up = True
        else:
            self.should_run = False

    def lower_excavator_response(self, message: Bool):
        self.get_logger().info("Lowering excavator")
        if self.exc_routine_val:
            return

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

    exc = Excavator()
    try:
        rclpy.spin(exc)
    except KeyboardInterrupt:
        pass
    finally:
        exc.destroy_node()
        rclpy.shutdown()
