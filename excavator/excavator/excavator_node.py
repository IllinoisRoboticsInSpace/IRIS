import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from linear_actuator import LinearActuator

PHASE_INIT = 0
PHASE_BACK = 1
PHASE_LOWER = 2
PHASE_DIG = 3
PHASE_RAISE = 4
PHASE_STORE = 5

class Excavator(Node):
    def __init__(self):
        super().__init__('Excavator')

        self.linear_actuator = LinearActuator('S')

        self.exc_routine_publisher = self.create_publisher(
            Bool, 'exc_routine', 10)
        self.exc_routine_repeated_publisher = self.create_publisher(
            Bool, 'exc_routine_repeated', 10)
        self.tread_speed_left_exc_routine_publisher = self.create_publisher(
            Float32, 'tread_speed_left_exc_routine', 10)
        self.tread_speed_right_exc_routine_publisher = self.create_publisher(
            Float32, 'tread_speed_right_exc_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)

        self.raise_excavator_teleop_subscriber = self.create_subscription(
            Bool, 'raise_scooper_teleop', self.raise_excavator_response, 10)
        self.lower_excavator_teleop_subscriber = self.create_subscription(
            Bool, 'lower_scooper_teleop', self.lower_excavator_response, 10)
        self.exc_routine_subscription = self.create_subscription(
            Bool, 'exc_routine', self.exc_routine_response, 10)
        self.exc_routine_repeated_subscription = self.create_subscription(
            Bool, 'exc_routine_repeated', self.exc_routine_repeated_response, 10)

        self.exc_routine_val = False
        self.exc_routine_repeated_val = False
        self.should_run = False
        self.up = False
        
        self.tread_speed = 0.15
        self.twerk_speed = 0.30
        
        # timing based scooping routine
        self.lower_duration = 8.0 * 1e9
        self.back_duration = 0.75 * 1e9
        self.dig_duration = 3.5 * 1e9
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
            pass
        elif (self.routine_phase == PHASE_BACK):
            if (time.time_ns() - self.routine_phase_start_time > self.back_duration):
                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))

                self.routine_phase = PHASE_LOWER
                self.routine_phase_start_time = time.time_ns()

                self.up = False
                self.should_run = True
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
                self.routine_phase = PHASE_RAISE
                self.routine_phase_start_time = time.time_ns()

                self.up = True
                self.should_run = True
        elif (self.routine_phase == PHASE_RAISE):
            if (time.time_ns() - self.routine_phase_start_time > 5e8):
                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))

            if (time.time_ns() - self.routine_phase_start_time > self.raise_duration):
                self.routine_phase = PHASE_INIT if self.exc_routine_repeated_val else PHASE_STORE
                self.routine_phase_start_time = time.time_ns()
                
                self.up = False
                self.should_run = False

                if self.exc_routine_repeated_val:
                    self.exc_routine_repeated_val = False
                    self.exc_routine_repeated_publisher.publish(Bool(data=False))

        elif (self.routine_phase == PHASE_STORE):
            sign = 1 if ((time.time_ns() - self.routine_phase_start_time) // 2e8) % 2 == 1 else -1
            self.tread_speed_left_exc_routine_publisher.publish(Float32(data=sign*self.twerk_speed))
            self.tread_speed_right_exc_routine_publisher.publish(Float32(data=-sign*self.twerk_speed))
            if (time.time_ns() - self.routine_phase_start_time > self.store_duration):
                self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))
                
                if self.exc_routine_repeated_val:
                    self.routine_phase = PHASE_BACK
                    self.routine_phase_start_time = time.time_ns()

                    self.exc_routine_val = False
                    self.exc_routine_publisher.publish(Bool(data=False))

                    self.up = False
                    self.should_run = False

                    self.tread_speed_left_exc_routine_publisher.publish(Float32(data=-self.tread_speed))
                    self.tread_speed_right_exc_routine_publisher.publish(Float32(data=-self.tread_speed))
                else:
                    self.routine_phase = PHASE_INIT

                    self.exc_routine_val = False
                    self.exc_routine_publisher.publish(Bool(data=False))
                    self.exc_routine_repeated_val = False
                    self.exc_routine_publisher.publish(Bool(data=False))

                    self.up = False
                    self.should_run = False

                    self.tread_speed_left_exc_routine_publisher.publish(Float32(data=0.0))
                    self.tread_speed_right_exc_routine_publisher.publish(Float32(data=0.0))

    def exc_routine_response(self, message: Bool):
        self.exc_routine_val = message.data
        
        if message.data and not self.should_run:
            self.exc_routine_repeated_val = False
            self.exc_routine_repeated_publisher.publish(Bool(data=False))
 
            self.get_logger().info('Triggering scooping routine...')
 
            self.routine_phase = PHASE_BACK
            self.routine_phase_start_time = time.time_ns()
 
            self.up = False
            self.should_run = False

            self.tread_speed_left_exc_routine_publisher.publish(Float32(data=-self.tread_speed))
            self.tread_speed_right_exc_routine_publisher.publish(Float32(data=-self.tread_speed))

    def exc_routine_repeated_response(self, message: Bool):
        self.exc_routine_repeated_val = message.data

        if message.data and not self.should_run:
            self.exc_routine_val = False
            self.exc_routine_publisher.publish(Bool(data=False))

            self.get_logger().info('Triggering scooping routine repeated...')
 
            self.routine_phase = PHASE_BACK
            self.routine_phase_start_time = time.time_ns()
 
            self.up = False
            self.should_run = False

            self.tread_speed_left_exc_routine_publisher.publish(Float32(data=-self.tread_speed))
            self.tread_speed_right_exc_routine_publisher.publish(Float32(data=-self.tread_speed))

    def raise_excavator_response(self, message: Bool):
        self.get_logger().info(f"Raise excavator {message}")
        if self.exc_routine_val or self.exc_routine_repeated_val:
            self.routine_phase = PHASE_INIT
            self.exc_routine_val = False
            self.exc_routine_publisher.publish(Bool(data=False))
            self.exc_routine_repeated_val = False
            self.exc_routine_repeated_publisher.publish(Bool(data=False))

        if message.data:
            self.should_run = True
            self.up = True
        else:
            self.should_run = False

    def lower_excavator_response(self, message: Bool):
        self.get_logger().info(f"Lower excavator {message}")
        if self.exc_routine_val or self.exc_routine_repeated_val:
            self.routine_phase = PHASE_INIT
            self.exc_routine_val = False
            self.exc_routine_publisher.publish(Bool(data=False))
            self.exc_routine_repeated_val = False
            self.exc_routine_repeated_publisher.publish(Bool(data=False))

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
