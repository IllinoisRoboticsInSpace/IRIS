import serial
SERIAL_BUFFER_BYTES = 64

#Make new proto message for sending motor config and turnmotor message

class MotorDriver:

    
    def __init__(self, debugMode: bool = False, port = '/dev/ttyACM0', baudrate = 115200, timeout = .1) -> None:
        self.debugState = False
        self.serialLine = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
        # List of Motor configs
        self.motorConfigs = []




        
        

    def setDebugMode(self, toggle: bool):
        #Do last due to threading
        pass

    def sendMotorConfig(self):
        pass

    def stopMotors(self):
        #Separate op code to turn off all motors
        pass

    def turnMotor(self, motorID: int, output: int):
        pass    


class MotorConfig: 

    def __init__(self, motorID: int, serialPin: int, motorNum: int, address: int, inverted: bool = False):
        self.motorID = motorID
        self.serialPin = serialPin
        self.motorNum = motorNum
        self.address = address
        self.inverted = inverted

