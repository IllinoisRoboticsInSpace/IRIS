import Jetson.GPIO as GPIO
import time

PWM_PIN = 32  # Adjust based on your physical wiring
DIR_PIN = 31  # Adjust based on your physical wiring

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Initialize PWM at 1000Hz
pwm_ctrl = GPIO.PWM(PWM_PIN, 1000)
pwm_ctrl.start(0) # Start with 0% duty cycle

val = 0.5
        
# Clamp value between -1 and 1
val = max(min(val, 1.0), -1.0)

# Determine Direction
if val >= 0:
    GPIO.output(DIR_PIN, GPIO.HIGH)
else:
    GPIO.output(DIR_PIN, GPIO.LOW)
    
# Calculate Duty Cycle (0 to 100)
duty_cycle = abs(val) * 100
pwm_ctrl.ChangeDutyCycle(duty_cycle)

print(f'Setting speed to: {val}')

time.sleep(0.5)

pwm_ctrl.ChangeDutyCycle(0)