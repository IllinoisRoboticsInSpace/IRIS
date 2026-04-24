import gpiod
from gpiod.line import Direction, Value
import time
import threading


class LinearActuator:
    def __init__(self, gpio_chip: str, dir_line: int, pwm_line: int):
        # self.pwm_freq = 1000  # Hz
        # self.pwm_period = 1.0 / self.pwm_freq

        # self.stop_event = threading.Event()
        # self.thread_active = False
        # self.motor_thread: threading.Thread

        self.is_running = False

        self.gpio_chip = gpiod.Chip(gpio_chip)

        self.dir_line = chip.get_line(dir_line)
        self.pwm_line = chip.get_line(pwm_line)

        self.dir_line.request(consumer="mdd10a", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.pwm_line.request(consumer="mdd10a", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

    def release(self):
        self.dir_line.set_value(0);
        self.pwm_line.set_value(0);

        self.dir_line.release()
        self.pwm_line.release()

        self.chip.close()

    def set_direction(self, forward: bool):
        self.dir_line.set_value(0 if forward else 1)

    def run_motor(self, on: bool):
        if (on == self.is_running):
            return

        self.is_running = on

        # if self.thread_active:
        #     self.stop_event.set()
        #     self.motor_thread.join()
        #     self.thread_active = False

        # duty = max(0.0, min(1.0, duty))

        self.pwm_line.set_value(1 if on else 0)

        # if (duty == 0.0):
        #     self.lines.set_value(self.pwm_line, Value.INACTIVE)
        # elif (duty == 1.0):
        #     self.lines.set_value(self.pwm_line, Value.ACTIVE)
        # else:
        #     on_time = self.pwm_period * duty
        #     off_time = self.pwm_period * (1.0 - duty)

        #     def motor_lambda(): self._motor_function(duty, on_time, off_time)
        #     self.motor_thread = threading.Thread(target=motor_lambda)
        #     self.thread_active = True

    # def _motor_function(self, duty: float, on_time: float, off_time: float):
    #     while not self.stop_event.isSet():
    #         self.lines.set_value(self.pwm_line, Value.ACTIVE)
    #         time.sleep(on_time)

    #         self.lines.set_value(self.pwm_line, Value.INACTIVE)
    #         time.sleep(off_time)
