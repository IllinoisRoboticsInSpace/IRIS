import serial
from motor_driver.generated import commands_pb2
SERIAL_BUFFER_BYTES = 64 # What is this for
MIN_MOTOR_ID = 0
MAX_MOTOR_ID = 3

class MotorConfig: 

    def __init__(self, motorID: int, serialPin: int, motorNum: int, address: int, inverted: bool = False):
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")
        if motorNum < 1 or motorNum > 2:
            raise ValueError("motorNum must be between 1 and 2")
        self.motorID = motorID
        self.serialPin = serialPin
        self.motorNum = motorNum
        self.address = address
        self.inverted = inverted

class MotorDriver:

    
    def __init__(self, debugMode: bool = False, port = '/dev/ttyACM0', baudrate = 115200, timeout = .1) -> None:
        self.debugState = False
        self.serialLine = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
        # List of Motor configs
        self.motorConfigs = [None] * MAX_MOTOR_ID

    def initMotorDriver(self):
        pass # Send all configurations?
        
    def resetDevice(self):
        pass # Send Arduino a reset command?
        # Reset Arduino to original state

    def setDebugMode(self, toggle: bool):
        #Do last due to threading NOT processing
        #While loop reading same serial line
        #Global variable to shut it down
        # Test in different area
        pass

    def setMotorConfig(self, config: MotorConfig):
        self.motorConfigs[config.motorID] = config

    def sendMotorConfig(self, motorID: int):
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")

        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.CONFIG_MOTOR
        message.configData.motorID = self.motorConfigs[motorID].motorID
        self.serialLine.write(message.SerializeToString())
        
        message.configData.serialPin = self.motorConfigs[motorID].serialPin
        self.serialLine.write(message.SerializeToString())

        message.configData.motorNum = self.motorConfigs[motorID].motorNum
        self.serialLine.write(message.SerializeToString())

        message.configData.address = self.motorConfigs[motorID].address
        self.serialLine.write(message.SerializeToString())

        message.configData.inverted = self.motorConfigs[motorID].inverted
        self.serialLine.write(message.SerializeToString())

    def stopMotors(self):
        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.STOP_ALL_MOTORS
        self.serialLine.write(message.SerializeToString())

    def turnMotor(self, motorID: int, output: float):
        if (output < -1.0 or output > 1.0):
            raise ValueError("output must be between -1.0 and 1.0")
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")

        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.TURN_MOTOR
        message.motorCommand.percentSpeed = output
        message.motorCommand.motorID = motorID

        self.serialLine.write(message.SerializeToString())
