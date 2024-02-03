import serial
import threading
import time
from motor_driver.generated import commands_pb2
FIXED_RECEIVED_MESSAGE_LENGTH = 16 # The number of bytes of a message received from host
SERIAL_BUFFER_BYTES = 64 # What is this for
MIN_MOTOR_ID = 0
MAX_MOTOR_ID = 3

class Serial_Reader(threading.Thread):
    #Do last due to threading NOT processing
    #While loop reading same serial line
    
    #Have a toggle that when on, will print raw data to terminal, otherwise will just ignore it
    #Copy the one from the arduino side
    def __init__(self, serialObj: serial):
        super().__init__()
        self._stop_event = threading.Event()
        self.serialLine = serialObj
        self.debugState = False

    def run(self):
        while not self._stop_event.is_set():
            echoed_message = self.serialLine.read(FIXED_RECEIVED_MESSAGE_LENGTH)
            if self.debugState == True:
                print(echoed_message.hex()) 

    def debug_flag(self, toggle: bool):
        self.debugState = toggle

    def stop(self):
        self._stop_event.set()



class MotorConfig: 

    def __init__(self, motorID: int, serialLine: int, motorNum: int, address: int, inverted: bool = False):
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")
        if motorNum < 1 or motorNum > 2:
            raise ValueError("motorNum must be between 1 and 2")
        if serialLine < 0 or serialLine > 2:
            raise ValueError("serialLine must be between 0 and 2")
        self.motorID = motorID
        self.serialLine = serialLine
        self.motorNum = motorNum
        self.address = address
        self.inverted = inverted




class MotorDriver:

    def __init__(self, debugMode: bool = False, port = '/dev/ttyACM0', baudrate = 115200, timeout = .1) -> None:
        self.serialLine = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
        # List of Motor configs
        self.motorConfigs = [None] * MAX_MOTOR_ID
        #Start debug thread here?
        self.debugPrinter = Serial_Reader(self.serialLine)
        self.setDebugMode(debugMode)
        self.debugPrinter.start()
        
    def __del__(self):
        self.debugPrinter.stop()
        self.debugPrinter.join()

    def initMotorDriver(self):
        pass # Send all configurations?
        
    def resetDevice(self):
        pass # Send Arduino a reset command?
        # Reset Arduino to original state

    def setDebugMode(self, toggle: bool):
        self.debugPrinter.debug_flag(toggle)
        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.SET_DEBUG_MODE

        if toggle == True:
            message.debugMode.enabled = True
        else: 
            message.debugMode.enabled = False

        self.serialLine.write(message.SerializeToString())

    def setMotorConfig(self, config: MotorConfig):
        self.motorConfigs[config.motorID] = config

    def sendMotorConfig(self, motorID: int):
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")

        message = commands_pb2.Serial_Message()
        message.opcode = commands_pb2.CONFIG_MOTOR
        message.Sabertooth_Config_Data.motorID = self.motorConfigs[motorID].motorID
        message.Sabertooth_Config_Data.enabled = False
        self.serialLine.write(message.SerializeToString())
        
        message.Sabertooth_Config_Data.serialLine = self.motorConfigs[motorID].serialLine
        self.serialLine.write(message.SerializeToString())

        message.Sabertooth_Config_Data.motorNum = self.motorConfigs[motorID].motorNum
        self.serialLine.write(message.SerializeToString())

        message.Sabertooth_Config_Data.address = self.motorConfigs[motorID].address
        self.serialLine.write(message.SerializeToString())

        message.Sabertooth_Config_Data.inverted = self.motorConfigs[motorID].inverted
        self.serialLine.write(message.SerializeToString())

        message.Sabertooth_Config_Data.enabled = True
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
        message.motorCommand.percentOutput = output
        message.motorCommand.motorID = motorID

        self.serialLine.write(message.SerializeToString())


