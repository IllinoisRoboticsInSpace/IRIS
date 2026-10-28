from enum import Enum, auto
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from linear_actuator import LinearActuator


class DepositPhase(Enum):
    IDLE = auto()
    DRIVE_IN = auto()
    RAISE = auto()
    HOLD = auto()
    LOWER = auto()
    DRIVE_OUT = auto()


class ActuatorMode(Enum):
    STOPPED = auto()
    RAISING = auto()
    LOWERING = auto()


class Depositer(Node):
    def __init__(self):
        super().__init__('Depositer')

        self.linear_actuator = LinearActuator('D')

        self.dep_routine_publisher = self.create_publisher(
            Bool, 'dep_routine', 10)
        self.tread_speed_left_dep_routine_publisher = self.create_publisher(
            Float32, 'tread_speed_left_dep_routine', 10)
        self.tread_speed_right_dep_routine_publisher = self.create_publisher(
            Float32, 'tread_speed_right_dep_routine', 10)

        self.timer = self.create_timer(0.1, self.timer_response)

        self.raise_dumper_teleop_subscriber = self.create_subscription(
            Bool, 'raise_dumper_teleop', self.raise_dumper_response, 10)
        self.lower_dumper_teleop_subscriber = self.create_subscription(
            Bool, 'lower_dumper_teleop', self.lower_dumper_response, 10)
        self.dep_routine_subscription = self.create_subscription(
            Bool, 'dep_routine', self.dep_routine_response, 10)

        self.actuator_mode = ActuatorMode.STOPPED
        self.tread_speed = 0.15

        self.drive_duration_ns = 3.5 * 1e9
        self.lower_duration_ns = 40.0 * 1e9
        self.hold_duration_ns = 2.0 * 1e9
        self.raise_duration_ns = 45.0 * 1e9

        self.routine_phase = DepositPhase.IDLE
        self.routine_phase_start_time_ns = time.monotonic_ns()

    def timer_response(self):
        self._apply_actuator_mode()

        now_ns = time.monotonic_ns()
        elapsed_ns = now_ns - self.routine_phase_start_time_ns

        if self.routine_phase is DepositPhase.DRIVE_IN:
            if elapsed_ns > self.drive_duration_ns:
                self._publish_tread_speed(0.0)
                self._enter_phase(DepositPhase.RAISE, now_ns)
                self.actuator_mode = ActuatorMode.RAISING
        elif self.routine_phase is DepositPhase.RAISE:
            if elapsed_ns > self.raise_duration_ns:
                self._enter_phase(DepositPhase.HOLD, now_ns)
                self.actuator_mode = ActuatorMode.STOPPED
        elif self.routine_phase is DepositPhase.HOLD:
            if elapsed_ns > self.hold_duration_ns:
                self._enter_phase(DepositPhase.LOWER, now_ns)
                self.actuator_mode = ActuatorMode.LOWERING
        elif self.routine_phase is DepositPhase.LOWER:
            if elapsed_ns > self.lower_duration_ns:
                self._enter_phase(DepositPhase.DRIVE_OUT, now_ns)
                self.actuator_mode = ActuatorMode.LOWERING
                self._publish_tread_speed(-self.tread_speed)
        elif self.routine_phase is DepositPhase.DRIVE_OUT:
            if elapsed_ns > self.drive_duration_ns:
                self._publish_tread_speed(0.0)
                self._enter_phase(DepositPhase.IDLE, now_ns)
                self.actuator_mode = ActuatorMode.STOPPED
                self._publish_routine_status(False)

    def _apply_actuator_mode(self):
        if self.actuator_mode is ActuatorMode.STOPPED:
            self.linear_actuator.run_motor(False)
            return

        self.linear_actuator.set_direction(self.actuator_mode is ActuatorMode.RAISING)
        self.linear_actuator.run_motor(True)

    def _enter_phase(self, phase: DepositPhase, now_ns: int):
        self.routine_phase = phase
        self.routine_phase_start_time_ns = now_ns

    def _publish_tread_speed(self, speed: float):
        self.tread_speed_left_dep_routine_publisher.publish(Float32(data=speed))
        self.tread_speed_right_dep_routine_publisher.publish(Float32(data=speed))

    def _publish_routine_status(self, active: bool):
        self.dep_routine_publisher.publish(Bool(data=active))

    def _cancel_routine(self):
        if self.routine_phase is DepositPhase.IDLE:
            return

        self._enter_phase(DepositPhase.IDLE, time.monotonic_ns())
        self._publish_tread_speed(0.0)
        self._publish_routine_status(False)

    def dep_routine_response(self, message: Bool):
        if not message.data or self.routine_phase is not DepositPhase.IDLE:
            return

        if self.actuator_mode is not ActuatorMode.STOPPED:
            self._publish_routine_status(False)
            return

        self.get_logger().info('Triggering dumping routine...')
        now_ns = time.monotonic_ns()
        self._enter_phase(DepositPhase.DRIVE_IN, now_ns)
        self.actuator_mode = ActuatorMode.STOPPED
        self._publish_tread_speed(self.tread_speed)

    def raise_dumper_response(self, message: Bool):
        self.get_logger().info(f'Raise dumper {message}')
        self._cancel_routine()
        self.actuator_mode = ActuatorMode.RAISING if message.data else ActuatorMode.STOPPED

    def lower_dumper_response(self, message: Bool):
        self.get_logger().info(f'Lower dumper {message}')
        self._cancel_routine()
        self.actuator_mode = ActuatorMode.LOWERING if message.data else ActuatorMode.STOPPED

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
