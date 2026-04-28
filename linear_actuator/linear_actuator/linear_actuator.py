import gpiod
from gpiod.line import Direction, Value
import time
import threading
import serial

class LinearActuator:
    def __init__(self, id: str):
        # self.pwm_freq = 1000  # Hz
        # self.pwm_period = 1.0 / self.pwm_freq

        # self.stop_event = threading.Event()
        # self.thread_active = False
        # self.motor_thread: threading.Thread

        self.is_running = False

        self.id = id

        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    def release(self):
        self.serial.close()

    def set_direction(self, forward: bool):
        # self.dir_line.set_value(0 if forward else 1)
        if forward:
            self.send_serial_cmd('M' + id + 'DT')
        else:
            self.send_serial_cmd('M' + id + 'DF')

    def run_motor(self, on: bool):
        if (on == self.is_running):
            return

        self.is_running = on

        # if self.thread_active:
        #     self.stop_event.set()
        #     self.motor_thread.join()
        #     self.thread_active = False

        # duty = max(0.0, min(1.0, duty))

        # self.pwm_line.set_value(1 if on else 0)
        if on:
            self.send_serial_cmd('M' + id + 'PT')

        else:
            self.send_serial_cmd('M' + id + 'PF')

        
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

    def send_serial_cmd(self, command: str):
        if len(command) != 4 or command[0] != 'M':
            print("Invalid command format")
            return
    
        try:
            self.serial.write(command.encode())
            print(f"Sent: {command}")
        except serial.SerialException as e:
            print(f'Error: {e}')