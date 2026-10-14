"""Timed excavator controller using the existing IRIS topics.

False on the active routine topic cancels that routine immediately.
Repeated excavation includes STORE on every cycle and runs until canceled.
Hardware feedback, serial acknowledgments and drive arbitration are external.
"""
import time

import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
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
        self.exc_routine_val = False
        self.exc_routine_repeated_val = False
        self.stop_requested = False
        self.raise_requested = False
        self.lower_requested = False
        self.should_run = False
        self.up = False
        self.tread_speed = 0.15
        self.twerk_speed = 0.30
        self.lower_duration = 8.0 * 1e9
        self.back_duration = 0.75 * 1e9
        self.dig_duration = 3.5 * 1e9
        self.raise_duration = 12.0 * 1e9
        self.store_duration = 2.0 * 1e9
        self.routine_phase = PHASE_INIT
        self.routine_phase_start_time = 0

        self.exc_routine_publisher = self.create_publisher(Bool, 'exc_routine', 10)
        self.exc_routine_repeated_publisher = self.create_publisher(
            Bool, 'exc_routine_repeated', 10)
        self.tread_speed_left_exc_routine_publisher = self.create_publisher(
            Float32, 'tread_speed_left_exc_routine', 10)
        self.tread_speed_right_exc_routine_publisher = self.create_publisher(
            Float32, 'tread_speed_right_exc_routine', 10)
        self.raise_excavator_teleop_subscriber = self.create_subscription(
            Bool, 'raise_scooper_teleop', self.raise_excavator_response, 10)
        self.lower_excavator_teleop_subscriber = self.create_subscription(
            Bool, 'lower_scooper_teleop', self.lower_excavator_response, 10)
        self.exc_routine_subscription = self.create_subscription(
            Bool, 'exc_routine', self.exc_routine_response, 10)
        self.exc_routine_repeated_subscription = self.create_subscription(
            Bool, 'exc_routine_repeated', self.exc_routine_repeated_response, 10)
        self.stop_subscription = self.create_subscription(
            Bool, 'stop_flag_teleop', self.stop_response, 10)
        # Physical hardware timing must not depend on wall-clock adjustments
        # or a paused simulation clock.
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.timer_response, clock=self.timer_clock)

    def _publish_treads(self, left, right):
        self.tread_speed_left_exc_routine_publisher.publish(Float32(data=float(left)))
        self.tread_speed_right_exc_routine_publisher.publish(Float32(data=float(right)))

    def _enter_phase(self, phase, now):
        self.routine_phase = phase
        self.routine_phase_start_time = now
        self.get_logger().info(f'Excavation phase: {phase}')

    def _apply_actuator(self):
        if not self.should_run:
            self.linear_actuator.run_motor(False)
            return
        # Issue stop before reversing an actuator that the driver says is on.
        # Any required reversal delay must be specified by the hardware team.
        if (self.linear_actuator.is_running
                and self.linear_actuator.is_forward != self.up):
            self.linear_actuator.run_motor(False)
        self.linear_actuator.set_direction(self.up)
        self.linear_actuator.run_motor(True)

    def _finish_routine(self):
        self.routine_phase = PHASE_INIT
        self.should_run = False
        self.exc_routine_val = False
        self.exc_routine_repeated_val = False
        self._apply_actuator()
        self._publish_treads(0.0, 0.0)
        self.exc_routine_publisher.publish(Bool(data=False))
        self.exc_routine_repeated_publisher.publish(Bool(data=False))

    def _start_routine(self, repeated):
        if (self.stop_requested or self.routine_phase != PHASE_INIT
                or self.raise_requested or self.lower_requested):
            self.get_logger().warning('Excavation start rejected: stopped or busy.')
            # Restore the existing controller's sticky request state.
            if repeated and not self.exc_routine_repeated_val:
                self.exc_routine_repeated_publisher.publish(Bool(data=False))
            elif not repeated and not self.exc_routine_val:
                self.exc_routine_publisher.publish(Bool(data=False))
            return
        self.exc_routine_val = not repeated
        self.exc_routine_repeated_val = repeated
        self.should_run = False
        self._apply_actuator()
        self._enter_phase(PHASE_BACK, time.monotonic_ns())
        self._publish_treads(-self.tread_speed, -self.tread_speed)

    def timer_response(self):
        now = time.monotonic_ns()
        if self.stop_requested:
            self.should_run = False
            self._apply_actuator()
            return

        if self.routine_phase == PHASE_INIT:
            self.should_run = self.raise_requested != self.lower_requested
            self.up = self.raise_requested
            self._apply_actuator()
            # Do not publish idle tread zeros: the supplied drive node accepts
            # excavator commands unconditionally, which would override teleop.
            return

        elapsed = now - self.routine_phase_start_time
        next_phase = None
        if self.routine_phase == PHASE_BACK and elapsed >= self.back_duration:
            next_phase = PHASE_LOWER
        elif self.routine_phase == PHASE_LOWER and elapsed >= self.lower_duration:
            next_phase = PHASE_DIG
        elif self.routine_phase == PHASE_DIG and elapsed >= self.dig_duration:
            next_phase = PHASE_RAISE
        elif self.routine_phase == PHASE_RAISE and elapsed >= self.raise_duration:
            next_phase = PHASE_STORE
        elif self.routine_phase == PHASE_STORE and elapsed >= self.store_duration:
            if self.exc_routine_repeated_val:
                next_phase = PHASE_BACK
            else:
                self._finish_routine()
                return
        if next_phase is not None:
            self._enter_phase(next_phase, now)
            elapsed = 0

        self.should_run = self.routine_phase in (PHASE_LOWER, PHASE_RAISE)
        self.up = self.routine_phase == PHASE_RAISE
        left = right = 0.0
        if self.routine_phase == PHASE_BACK:
            left = right = -self.tread_speed
        elif self.routine_phase == PHASE_DIG:
            left = right = self.tread_speed
        elif self.routine_phase == PHASE_RAISE and elapsed < 5.0:
            left = right = self.tread_speed
        elif self.routine_phase == PHASE_STORE:
            sign = 1 if (elapsed // 2.0) % 2 else -1
            left, right = sign * self.twerk_speed, -sign * self.twerk_speed
        self._apply_actuator()
        self._publish_treads(left, right)

    def exc_routine_response(self, message):
        if message.data:
            self._start_routine(repeated=False)
        elif self.exc_routine_val:
            self._finish_routine()

    def exc_routine_repeated_response(self, message):
        if message.data:
            self._start_routine(repeated=True)
        elif self.exc_routine_repeated_val:
            self._finish_routine()

    def raise_excavator_response(self, message):
        if self.stop_requested:
            return
        self.raise_requested = message.data
        if message.data and self.routine_phase != PHASE_INIT:
            self._finish_routine()
        # Resolve both button states together on the next timer callback.

    def lower_excavator_response(self, message):
        if self.stop_requested:
            return
        self.lower_requested = message.data
        if message.data and self.routine_phase != PHASE_INIT:
            self._finish_routine()

    def stop_response(self, message):
        self.stop_requested = message.data
        self.raise_requested = False
        self.lower_requested = False
        if message.data:
            self._finish_routine()
        # Releasing stop does not resume an interrupted routine or held input.

    def destroy_node(self):
        self.timer.cancel()
        try:
            self.should_run = False
            self._apply_actuator()
            self._publish_treads(0.0, 0.0)
        finally:
            try:
                self.linear_actuator.release()
            finally:
                super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    exc = None
    try:
        exc = Excavator()
        rclpy.spin(exc)
    except KeyboardInterrupt:
        pass
    finally:
        if exc is not None:
            exc.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
