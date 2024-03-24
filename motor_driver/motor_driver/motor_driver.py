import serial
from serial.threaded import ReaderThread, LineReader
import threading
import time
from motor_driver.generated import commands_pb2
import logging
from time import sleep

try:
    import queue
except ImportError:
    import Queue as queue

FIXED_RECEIVED_MESSAGE_LENGTH = 16 # The number of bytes of a message received from host
SERIAL_BUFFER_BYTES = 64 # What is this for
MIN_MOTOR_ID = 0
MAX_MOTOR_ID = 3
DEBUG_MODE = True


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
        # self.debug_mode = True

    def stop(self):
        self.alive = False
        self.events.put(None)

    def _run_event(self):
        while self.alive:
            try:
                event = self.events.get()
                if DEBUG_MODE == True:
                    print("event received: ", event)
            except:
                logging.exception("could not run event")

    # def set_debug_mode(self, state: bool):
    #     self.debug_mode = state

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
        #Start debug thread here?
        # self.debugPrinter = SerialReader(self.serialLine)
        self.debugReader = ReaderThread(self.serialLine, SerialReader)
        self.setDebugMode(debugMode)
        # self.debugPrinter.start()
        self.debugReader.start()
        
    # def __del__(self):
    #     # self.debugPrinter.stop()
    #     # self.debugPrinter.join()
    #     self.debugReader.stop()

    def initMotorDriver(self):
        # send all motor configs
        for motor_id in range(len(self.motorConfigs)):
            if self.motorConfigs[motor_id] is not None:
                self.sendMotorConfig(motor_id)
        # pass # Send all configurations?
        
    def resetDevice(self):
        self.serialLine.setDTR(False)
        sleep(0.022)
        self.serialLine.setDTR(True)
        # pass # Send Arduino a reset command?
        # Reset Arduino to original state

    def setDebugMode(self, toggle: bool):
        # self.debugPrinter.debug_flag(toggle)
        # self.debugReader.set_debug_mode(toggle)
        DEBUG_MODE = toggle
        message = commands_pb2.Serial_Message_To_Arduino()
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

        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.CONFIG_MOTOR
        message.sabertoothConfigData.motorID = self.motorConfigs[motorID].motorID
        message.sabertoothConfigData.enabled = False
        self.serialLine.write(message.SerializeToString())
        
        message.sabertoothConfigData.serialLine = self.motorConfigs[motorID].serialLine
        self.serialLine.write(message.SerializeToString())

        message.sabertoothConfigData.motorNum = self.motorConfigs[motorID].motorNum
        self.serialLine.write(message.SerializeToString())

        message.sabertoothConfigData.address = self.motorConfigs[motorID].address
        self.serialLine.write(message.SerializeToString())

        message.sabertoothConfigData.inverted = self.motorConfigs[motorID].inverted
        self.serialLine.write(message.SerializeToString())

        message.sabertoothConfigData.enabled = True
        self.serialLine.write(message.SerializeToString())

    def stopMotors(self):
        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.STOP_ALL_MOTORS
        self.serialLine.write(message.SerializeToString())

    def turnMotor(self, motorID: int, output: float):
        if (output < -1.0 or output > 1.0):
            raise ValueError("output must be between -1.0 and 1.0")
        if motorID < MIN_MOTOR_ID or motorID > MAX_MOTOR_ID:
            raise ValueError("Invalid Motor ID")

        message = commands_pb2.Serial_Message_To_Arduino()
        message.opcode = commands_pb2.TURN_MOTOR
        message.motorCommand.percentOutput = output
        message.motorCommand.motorID = motorID

        self.serialLine.write(message.SerializeToString())