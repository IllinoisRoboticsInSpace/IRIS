import gpiod
import time
import signal
import sys
from gpiod.line import Direction, Value

# ==============================
# USER CONFIG
# ==============================
GPIO_CHIP = "/dev/gpiochip0"
DIR_LINE = 144
PWM_LINE = 112

PWM_FREQUENCY = 1000  # Hz
PWM_PERIOD = 1.0 / PWM_FREQUENCY

running = True

# ==============================
# CLEANUP / SIGNAL
# ==============================
def cleanup():
    try:
        lines.release()
    except Exception:
        pass

def signal_handler(sig, frame):
    global running
    running = False
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ==============================
# REQUEST GPIO LINES (v2)
# ==============================
lines = gpiod.request_lines(
    GPIO_CHIP,
    consumer="mdd10a",
    config={
        DIR_LINE: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
        PWM_LINE: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
    },
)

# ==============================
# MOTOR CONTROL
# ==============================
def set_direction(forward: bool):
    """Set direction using DIR_LINE"""
    lines.set_value(DIR_LINE, Value.ACTIVE if forward else Value.INACTIVE)

def run_motor(speed_percent: float, duration: float):
    """Run actuator using software PWM"""
    speed_percent = max(0, min(100, speed_percent))
    duty = speed_percent / 100.0

    on_time = PWM_PERIOD * duty
    off_time = PWM_PERIOD * (1.0 - duty)

    start = time.time()

    while running and (time.time() - start) < duration:
        if duty > 0:
            lines.set_value(PWM_LINE, Value.ACTIVE)
            time.sleep(on_time)
        if duty < 1:
            lines.set_value(PWM_LINE, Value.INACTIVE)
            time.sleep(off_time)

    lines.set_value(PWM_LINE, Value.INACTIVE)

# ==============================
# TEST
# ==============================
try:
    print("Extending actuator")
    set_direction(True)
    run_motor(70, 1)

    time.sleep(1)

    print("Retracting actuator")
    set_direction(False)
    run_motor(50, 1)

finally:
    cleanup()
