import serial
from motor_driver.generated import commands_pb2
SERIAL_BUFFER_BYTES = 64

#Make new proto message for sending motor config and turnmotor message


class MotorConfig: 

    def __init__(self, motorID: int, serialPin: int, motorNum: int, address: int, inverted: bool = False):
        if motorID < 1 or motorID > 4:
            raise ValueError("motorID must be between 1 and 4")
        if motorNum < 1 or motorNum > 2:
            raise ValueError("motorID must be between 1 and 2")
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
        self.motorConfigs = []
        # Where do we create the motor config objects
        # Where do we call the sendMotorConfig function




        
        

    def setDebugMode(self, toggle: bool):
        #Do last due to threading
        pass

    def sendMotorConfig(self, config: MotorConfig):
        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.CONFIG_MOTOR
        message.configData.motorID = config.motorID
        self.serialLine.write(message.SerializeToString())
        
        message.configData.serialPin = config.serialPin
        self.serialLine.write(message.SerializeToString())

        message.configData.motorNum = config.motorNum
        self.serialLine.write(message.SerializeToString())

        message.configData.address = config.address
        self.serialLine.write(message.SerializeToString())

        message.configData.inverted = config.inverted
        self.serialLine.write(message.SerializeToString())

    def stopMotors(self):
        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.STOP_ALL_MOTORS
        self.serialLine.write(message.SerializeToString())

    def turnMotor(self, motorID: int, output: int):
        if (output < -128 or output > 128):
            raise ValueError("output must be between -128 and 128")

        message = commands_pb2.Serial_Message()
        if motorID == 1:
            message.opcode = commands_pb2.TURN_MOTOR_1
        elif motorID == 2:
            message.opcode = commands_pb2.TURN_MOTOR_2
        elif motorID == 3:
            message.opcode = commands_pb2.TURN_MOTOR_3
        elif motorID == 4:
            message.opcode = commands_pb2.TURN_MOTOR_4
        else:
            raise ValueError("motorID must be between 1 and 4")
        message.motorCommand.percentSpeed = output

        self.serialLine.write(message.SerializeToString())
