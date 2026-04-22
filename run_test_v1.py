import gpiod
import time
import signal
import sys

# ==============================
# USER CONFIG
# ==============================
GPIO_CHIP = "/dev/gpiochip0"
DIR1_LINE = 112 # 105
PWM1_LINE = 122 # 106
DIR2_LINE = 85 # 43
PWM2_LINE = 50 # 133 

PWM_FREQUENCY = 1000  # Hz
PWM_PERIOD = 1.0 / PWM_FREQUENCY

running = True

# ==============================
# OPEN CHIP + LINES (v1 API)
# ==============================
chip = gpiod.Chip(GPIO_CHIP)

dir1_line = chip.get_line(DIR1_LINE)
pwm1_line = chip.get_line(PWM1_LINE)

dir2_line = chip.get_line(DIR2_LINE)
pwm2_line = chip.get_line(PWM2_LINE)

dir1_line.request(consumer="mdd10a", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
pwm1_line.request(consumer="mdd10a", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

dir2_line.request(consumer="mdd10a", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
pwm2_line.request(consumer="mdd10a", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

# ==============================
# CLEANUP / SIGNAL
# ==============================
def cleanup():
    try:
        pwm1_line.set_value(0)
        dir1_line.set_value(0)

        pwm2_line.set_value(0)
        dir2_line.set_value(0)
        
        pwm1_line.release()
        dir1_line.release()
       
        pwm2_line.release()
        dir2_line.release()
        
        chip.close()
    except Exception:
        pass

def signal_handler(sig, frame):
    global running
    running = False
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ==============================
# MOTOR CONTROL
# ==============================
def set_direction(forward: bool):
    """Set direction using DIR_LINE"""
    dir1_line.set_value(1 if forward else 0)
    dir2_line.set_value(1 if forward else 0)

def run_motor(speed_percent: float, duration: float):
    """Software PWM control"""
    speed_percent = max(0, min(100, speed_percent))
    duty = speed_percent / 100.0

    on_time = PWM_PERIOD * duty
    off_time = PWM_PERIOD * (1.0 - duty)

    start = time.time()

    while running and (time.time() - start) < duration:
        if duty > 0:
            pwm1_line.set_value(1)
            pwm2_line.set_value(1)
            time.sleep(on_time)

        if duty < 1:
            pwm1_line.set_value(0)
            pwm2_line.set_value(0)
            time.sleep(off_time)

        pwm1_line.set_value(1)
        pwm2_line.set_value(1)

    pwm1_line.set_value(0)
    pwm2_line.set_value(0)

def run_motor_locked_antiphase(speed_percent: float, duration: float):
    """Software PWM control"""
    speed_percent = max(0, min(100, speed_percent))
    duty = speed_percent / 100.0

    on_time = PWM_PERIOD * duty
    off_time = PWM_PERIOD * (1.0 - duty)

    start = time.time()
 
    pwm1_line.set_value(1)
    pwm2_line.set_value(1)
    
    while running and (time.time() - start) < duration:
        if duty > 0:
            dir1_line.set_value(1)
            dir2_line.set_value(1)
            time.sleep(on_time)

        if duty < 1:
            dir1_line.set_value(0)
            dir2_line.set_value(0)
            time.sleep(off_time)

        # pwm1_line.set_value(1)
        # pwm2_line.set_value(1)

    pwm1_line.set_value(0)
    pwm2_line.set_value(0)


# ==============================
# TEST
# ==============================
try:
    print("Extending actuator")
   # set_direction(False)
    run_motor_locked_antiphase(70, 15)
    
    time.sleep(1)

    print("Retracting actuator")
   # set_direction(True)
    run_motor_locked_antiphase(30, 15)

finally:
    cleanup()


#if __name__ == "__main__":
#    print("FORCE PWM1 HIGH")
#    pwm1_line.set_value(1)
#    time.sleep(10)
#    pwm1_line.set_value(0)
