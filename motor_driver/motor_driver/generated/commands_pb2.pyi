from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

CONFIG_ENCODER: Opcode_To_Arduino
CONFIG_MOTOR: Opcode_To_Arduino
CONFIG_PID: Opcode_To_Arduino
DEBUG_MESSAGE: Opcode_To_Jetson
DESCRIPTOR: _descriptor.FileDescriptor
ENCODER_COUNT: Opcode_To_Jetson
FOUR0: LatchMode
FOUR3: LatchMode
GET_ENCODER_COUNT: Opcode_To_Arduino
SET_DEBUG_MODE: Opcode_To_Arduino
SET_MOTOR_PID: Opcode_To_Arduino
SET_PID_SETPOINT: Opcode_To_Arduino
STOP_ALL_MOTORS: Opcode_To_Arduino
Serial1: SabertoothSerialLine
Serial2: SabertoothSerialLine
Serial3: SabertoothSerialLine
TURN_MOTOR: Opcode_To_Arduino
TWO03: LatchMode
ZERO_ENCODER: Opcode_To_Arduino

class Debug_Message(_message.Message):
    __slots__ = ["debug_data"]
    DEBUG_DATA_FIELD_NUMBER: _ClassVar[int]
    debug_data: str
    def __init__(self, debug_data: _Optional[str] = ...) -> None: ...

class Debug_Mode(_message.Message):
    __slots__ = ["enabled"]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    enabled: bool
    def __init__(self, enabled: bool = ...) -> None: ...

class Encoder_Config_Data(_message.Message):
    __slots__ = ["enabled", "encoderID", "inverted", "latchMode", "pinIn", "pinOut"]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    ENCODERID_FIELD_NUMBER: _ClassVar[int]
    INVERTED_FIELD_NUMBER: _ClassVar[int]
    LATCHMODE_FIELD_NUMBER: _ClassVar[int]
    PININ_FIELD_NUMBER: _ClassVar[int]
    PINOUT_FIELD_NUMBER: _ClassVar[int]
    enabled: bool
    encoderID: int
    inverted: bool
    latchMode: LatchMode
    pinIn: int
    pinOut: int
    def __init__(self, encoderID: _Optional[int] = ..., inverted: bool = ..., enabled: bool = ..., pinIn: _Optional[int] = ..., pinOut: _Optional[int] = ..., latchMode: _Optional[_Union[LatchMode, str]] = ...) -> None: ...

class Encoder_Count(_message.Message):
    __slots__ = ["encoderID", "tick_count"]
    ENCODERID_FIELD_NUMBER: _ClassVar[int]
    TICK_COUNT_FIELD_NUMBER: _ClassVar[int]
    encoderID: int
    tick_count: int
    def __init__(self, encoderID: _Optional[int] = ..., tick_count: _Optional[int] = ...) -> None: ...

class Encoder_Count_Request(_message.Message):
    __slots__ = ["encoderID"]
    ENCODERID_FIELD_NUMBER: _ClassVar[int]
    encoderID: int
    def __init__(self, encoderID: _Optional[int] = ...) -> None: ...

class PID_Config_Data(_message.Message):
    __slots__ = ["PID_ID", "enabled", "inverted", "kd", "ki", "kp", "motorID"]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    INVERTED_FIELD_NUMBER: _ClassVar[int]
    KD_FIELD_NUMBER: _ClassVar[int]
    KI_FIELD_NUMBER: _ClassVar[int]
    KP_FIELD_NUMBER: _ClassVar[int]
    MOTORID_FIELD_NUMBER: _ClassVar[int]
    PID_ID: int
    PID_ID_FIELD_NUMBER: _ClassVar[int]
    enabled: bool
    inverted: bool
    kd: float
    ki: float
    kp: float
    motorID: int
    def __init__(self, PID_ID: _Optional[int] = ..., inverted: bool = ..., enabled: bool = ..., motorID: _Optional[int] = ..., kp: _Optional[float] = ..., ki: _Optional[float] = ..., kd: _Optional[float] = ...) -> None: ...

class Sabertooth_Config_Data(_message.Message):
    __slots__ = ["address", "enabled", "inverted", "motorID", "motorNum", "serialLine"]
    ADDRESS_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    INVERTED_FIELD_NUMBER: _ClassVar[int]
    MOTORID_FIELD_NUMBER: _ClassVar[int]
    MOTORNUM_FIELD_NUMBER: _ClassVar[int]
    SERIALLINE_FIELD_NUMBER: _ClassVar[int]
    address: int
    enabled: bool
    inverted: bool
    motorID: int
    motorNum: int
    serialLine: SabertoothSerialLine
    def __init__(self, motorID: _Optional[int] = ..., inverted: bool = ..., enabled: bool = ..., motorNum: _Optional[int] = ..., serialLine: _Optional[_Union[SabertoothSerialLine, str]] = ..., address: _Optional[int] = ...) -> None: ...

class Serial_Message_To_Arduino(_message.Message):
    __slots__ = ["debugMode", "encoderConfigData", "encoderCountRequest", "motorCommand", "opcode", "pidConfigData", "sabertoothConfigData", "setPIDControl", "setPIDSetpoint", "zeroEncoderCommand"]
    DEBUGMODE_FIELD_NUMBER: _ClassVar[int]
    ENCODERCONFIGDATA_FIELD_NUMBER: _ClassVar[int]
    ENCODERCOUNTREQUEST_FIELD_NUMBER: _ClassVar[int]
    MOTORCOMMAND_FIELD_NUMBER: _ClassVar[int]
    OPCODE_FIELD_NUMBER: _ClassVar[int]
    PIDCONFIGDATA_FIELD_NUMBER: _ClassVar[int]
    SABERTOOTHCONFIGDATA_FIELD_NUMBER: _ClassVar[int]
    SETPIDCONTROL_FIELD_NUMBER: _ClassVar[int]
    SETPIDSETPOINT_FIELD_NUMBER: _ClassVar[int]
    ZEROENCODERCOMMAND_FIELD_NUMBER: _ClassVar[int]
    debugMode: Debug_Mode
    encoderConfigData: Encoder_Config_Data
    encoderCountRequest: Encoder_Count_Request
    motorCommand: Turn_Motor
    opcode: Opcode_To_Arduino
    pidConfigData: PID_Config_Data
    sabertoothConfigData: Sabertooth_Config_Data
    setPIDControl: Set_PID_Control
    setPIDSetpoint: Set_PID_Setpoint
    zeroEncoderCommand: Zero_Encoder_Command
    def __init__(self, opcode: _Optional[_Union[Opcode_To_Arduino, str]] = ..., sabertoothConfigData: _Optional[_Union[Sabertooth_Config_Data, _Mapping]] = ..., motorCommand: _Optional[_Union[Turn_Motor, _Mapping]] = ..., debugMode: _Optional[_Union[Debug_Mode, _Mapping]] = ..., pidConfigData: _Optional[_Union[PID_Config_Data, _Mapping]] = ..., setPIDSetpoint: _Optional[_Union[Set_PID_Setpoint, _Mapping]] = ..., setPIDControl: _Optional[_Union[Set_PID_Control, _Mapping]] = ..., encoderConfigData: _Optional[_Union[Encoder_Config_Data, _Mapping]] = ..., zeroEncoderCommand: _Optional[_Union[Zero_Encoder_Command, _Mapping]] = ..., encoderCountRequest: _Optional[_Union[Encoder_Count_Request, _Mapping]] = ...) -> None: ...

class Serial_Message_To_Jetson(_message.Message):
    __slots__ = ["debug_message", "encoder_count_data", "opcode"]
    DEBUG_MESSAGE_FIELD_NUMBER: _ClassVar[int]
    ENCODER_COUNT_DATA_FIELD_NUMBER: _ClassVar[int]
    OPCODE_FIELD_NUMBER: _ClassVar[int]
    debug_message: Debug_Message
    encoder_count_data: Encoder_Count
    opcode: Opcode_To_Jetson
    def __init__(self, opcode: _Optional[_Union[Opcode_To_Jetson, str]] = ..., debug_message: _Optional[_Union[Debug_Message, _Mapping]] = ..., encoder_count_data: _Optional[_Union[Encoder_Count, _Mapping]] = ...) -> None: ...

class Set_PID_Control(_message.Message):
    __slots__ = ["PID_ID", "in_control"]
    IN_CONTROL_FIELD_NUMBER: _ClassVar[int]
    PID_ID: int
    PID_ID_FIELD_NUMBER: _ClassVar[int]
    in_control: bool
    def __init__(self, PID_ID: _Optional[int] = ..., in_control: bool = ...) -> None: ...

class Set_PID_Setpoint(_message.Message):
    __slots__ = ["PID_ID", "setPoint"]
    PID_ID: int
    PID_ID_FIELD_NUMBER: _ClassVar[int]
    SETPOINT_FIELD_NUMBER: _ClassVar[int]
    setPoint: float
    def __init__(self, PID_ID: _Optional[int] = ..., setPoint: _Optional[float] = ...) -> None: ...

class Turn_Motor(_message.Message):
    __slots__ = ["motorID", "percentOutput"]
    MOTORID_FIELD_NUMBER: _ClassVar[int]
    PERCENTOUTPUT_FIELD_NUMBER: _ClassVar[int]
    motorID: int
    percentOutput: float
    def __init__(self, motorID: _Optional[int] = ..., percentOutput: _Optional[float] = ...) -> None: ...

class Zero_Encoder_Command(_message.Message):
    __slots__ = ["encoderID"]
    ENCODERID_FIELD_NUMBER: _ClassVar[int]
    encoderID: int
    def __init__(self, encoderID: _Optional[int] = ...) -> None: ...

class SabertoothSerialLine(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class LatchMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class Opcode_To_Arduino(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class Opcode_To_Jetson(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
