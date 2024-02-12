import pytest
from motor_driver import MotorDriver
from motor_driver import MotorConfig
from motor_driver import Serial_Reader

from motor_driver.generated import commands_pb2

def test_print():
    print("Hello World!")
    driver = MotorDriver(debugMode = True)
    driver.turnMotor(0, 0)
    # config = MotorConfig(0, 0, 1)
    # driver.setMotorConfig(config)
    # driver.sendMotorConfig(0)
    assert 1 == 0