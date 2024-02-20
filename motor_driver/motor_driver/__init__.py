import threading

from .motor_driver import MotorDriver
from .motor_driver import MotorConfig
from .motor_driver import Serial_Reader

MotorID = 3         # 1-3
numMotors = 2       # 1-2
SerialBaud = 9600   # Set at 9600 in C++ code but is 115200 in py ~?
SerialLine = 2      # 0-2
Address = 0x000000  # TODO Address of python controller?

config = MotorConfig(SerialLine, numMotors, Address)
Driver = MotorDriver()

Driver.resetDevice
Driver.initMotorDriver() # TODO
Driver.setMotorConfig(config) # TODO