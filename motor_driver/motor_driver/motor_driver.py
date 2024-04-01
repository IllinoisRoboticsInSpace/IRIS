import sys
import serial
from serial.threaded import ReaderThread, LineReader
import threading
import time
from motor_driver.generated import commands_pb2
import logging
from time import sleep
import string

try:
    import queue
except ImportError:
    import Queue as queue

# Modifyable
DEBUG_MODE = True

# Constant
SERIAL_BUFFER_BYTES = 64 # Max hardware serial buffer size

MIN_MOTOR_ID = 0
MAX_MOTOR_ID = 15 # Maximum number of motor ids 0 indexed
MAX_MOTOR_CONFIGS = (MAX_MOTOR_ID + 1)

MAX_ENCODER_ID = 14 # Maximum number of encoders ids 0 indexed
MAX_ENCODER_CONFIGS = (MAX_ENCODER_ID + 1)
DEFAULT_HOST_SERIAL_BAUD_RATE = 112500 # Baud rate of serial communication with host

#TODO: Write unit test to always check that this is valid
FIXED_RECEIVED_MESSAGE_LENGTH = 24 # The number of bytes of a message received from host
RECEIVED_COMMAND_BUFFER_SIZE = (FIXED_RECEIVED_MESSAGE_LENGTH * 2) # Size of commands buffer, data comes from host

FIXED_SEND_MESSAGE_LENGTH = 24 # The number of bytes of a message to send to host
SEND_COMMAND_BUFFER_SIZE = (FIXED_SEND_MESSAGE_LENGTH) # Size of buffer for data that goes to host

# Debug functionality message defines
MAX_DEBUG_STRING_SIZE_BYTES = 6

# string end char for queue
STR_END = "e"

# Size of serial buffer on arduino
ARDUINO_SERIAL_BUFFER_SIZE = 64

class SerialReader(LineReader):

    TERMINATOR = b'\r\n'

    def __init__(self):
        super(SerialReader, self).__init__()
        self.alive = True
        self.events = queue.Queue()
        self._event_thread = threading.Thread(target=self._run_event)
        self._event_thread.daemon = True
        self._event_thread.name = 'serialreader-event'
        self._event_thread.start()
        self.lock = threading.Lock()

    def stop(self):
        self.alive = False
        self.events.put(None)

    def _run_event(self):
        while self.alive:
            try:
                event = self.events.get()
                if DEBUG_MODE == True:
                    if(event == STR_END):
                        print("")
                    else:
                        print(f"{event}", end="")
            except:
                logging.exception("could not run event")

    def connection_made(self, transport: ReaderThread) -> None:
        super(SerialReader, self).connection_made(transport)
        print("connected, ready to receive data")
    
    def handle_line(self, line: str) -> None:
        if len(line) <= FIXED_RECEIVED_MESSAGE_LENGTH:
            self.events.put(line)
        else:
            line_chunks = [line[i:i+FIXED_RECEIVED_MESSAGE_LENGTH] for i in range(0, len(line), FIXED_RECEIVED_MESSAGE_LENGTH)]
            for chunk in line_chunks:
                self.events.put(chunk)
            self.events.put(STR_END)

    def connection_lost(self, exc: BaseException) -> None:
        super().connection_lost(exc)
        print("disconnected")


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
        self.debugReader = ReaderThread(self.serialLine, SerialReader)
        self.setDebugMode(debugMode)
        self.debugReader.start()
        sleep(1)

    def initMotorDriver(self):
        for motor_id in range(len(self.motorConfigs)):
            if self.motorConfigs[motor_id] is not None:
                self.sendMotorConfig(motor_id)
                print("motor config update")

        # self.turnMotor(0, 0.5)
        
    def resetDevice(self):
        self.serialLine.setDTR(False)
        sleep(0.022)
        self.serialLine.setDTR(True)

    def setDebugMode(self, toggle: bool):
        global DEBUG_MODE
        DEBUG_MODE = toggle

        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.SET_DEBUG_MODE

        if toggle == True:
            message.debugMode.enabled = True
        else: 
            message.debugMode.enabled = False

        # print(type(self.stringFill(message)))
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

    def stringFill(self, msg):
        serialized_str = msg.SerializeToString()
        # print(serialized_str)
        new_msg = serialized_str.ljust(FIXED_RECEIVED_MESSAGE_LENGTH, b'\x00')[:FIXED_RECEIVED_MESSAGE_LENGTH]
        # print(new_msg)
        return new_msg

    def setMotorConfig(self, config: MotorConfig):
        self.motorConfigs[config.motorID] = config

    def sendMotorConfig(self, motorID: int):
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")

        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.CONFIG_MOTOR
        message.sabertoothConfigData.motorID = self.motorConfigs[motorID].motorID
        message.sabertoothConfigData.enabled = False
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)
        
        sleep(1)
        message.sabertoothConfigData.serialLine = self.motorConfigs[motorID].serialLine
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

        sleep(1)
        message.sabertoothConfigData.motorNum = self.motorConfigs[motorID].motorNum
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

        sleep(1)
        message.sabertoothConfigData.address = self.motorConfigs[motorID].address
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

        sleep(1)
        message.sabertoothConfigData.inverted = self.motorConfigs[motorID].inverted
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

        sleep(1)
        message.sabertoothConfigData.enabled = True
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

        sleep(3)

    # Message will wait until there are bytes available to send
    def sendMessageBlocking(self, message):
        extended_bytes_data = self.stringFill(message)
        bytes_to_write = len(extended_bytes_data)
        # Wait till space is available
        while (not (ARDUINO_SERIAL_BUFFER_SIZE - self.serialLine.out_waiting > SEND_COMMAND_BUFFER_SIZE)):
            bytes_to_write = len(extended_bytes_data) #dummy function
        
        self.serialLine.write(extended_bytes_data)
        

    def stopMotors(self):
        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.STOP_ALL_MOTORS
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)

    def turnMotor(self, motorID: int, output: float):
        if (output < -1.0 or output > 1.0):
            raise ValueError("output must be between -1.0 and 1.0")
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")

        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.TURN_MOTOR
        message.motorCommand.percentOutput = output
        message.motorCommand.motorID = motorID

        # print(f"PYTHON DEBUG: {str(message)}: {len(message.SerializeToString())}")
        # print(f"PYTHON DEBUG: tyopeof{type(commands_pb2.TURN_MOTOR)}: {commands_pb2.TURN_MOTOR}")
        # print(f"Desired Data: {message.SerializeToString()}, Extended Data: {self.stringFill(message)}")
        # self.serialLine.write(self.stringFill(message))
        self.sendMessageBlocking(message)
        sleep(.1)