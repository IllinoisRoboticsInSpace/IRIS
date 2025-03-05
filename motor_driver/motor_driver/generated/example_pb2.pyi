import embedded_proto_options_pb2 as _embedded_proto_options_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
ONE: Example_Enum
TWO: Example_Enum
ZERO: Example_Enum

class A(_message.Message):
    __slots__ = ["a"]
    A_FIELD_NUMBER: _ClassVar[int]
    a: int
    def __init__(self, a: _Optional[int] = ...) -> None: ...

class Nested(_message.Message):
    __slots__ = ["nested_a"]
    NESTED_A_FIELD_NUMBER: _ClassVar[int]
    nested_a: A
    def __init__(self, nested_a: _Optional[_Union[A, _Mapping]] = ...) -> None: ...

class Oneof(_message.Message):
    __slots__ = ["bool_choice", "double_choice", "int32_choice"]
    BOOL_CHOICE_FIELD_NUMBER: _ClassVar[int]
    DOUBLE_CHOICE_FIELD_NUMBER: _ClassVar[int]
    INT32_CHOICE_FIELD_NUMBER: _ClassVar[int]
    bool_choice: bool
    double_choice: float
    int32_choice: int
    def __init__(self, bool_choice: bool = ..., int32_choice: _Optional[int] = ..., double_choice: _Optional[float] = ...) -> None: ...

class Optional(_message.Message):
    __slots__ = ["option"]
    OPTION_FIELD_NUMBER: _ClassVar[int]
    option: int
    def __init__(self, option: _Optional[int] = ...) -> None: ...

class Raw_Bytes(_message.Message):
    __slots__ = ["b"]
    B_FIELD_NUMBER: _ClassVar[int]
    b: bytes
    def __init__(self, b: _Optional[bytes] = ...) -> None: ...

class Repeated(_message.Message):
    __slots__ = ["repeated_int"]
    REPEATED_INT_FIELD_NUMBER: _ClassVar[int]
    repeated_int: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, repeated_int: _Optional[_Iterable[int]] = ...) -> None: ...

class Text(_message.Message):
    __slots__ = ["text_data"]
    TEXT_DATA_FIELD_NUMBER: _ClassVar[int]
    text_data: str
    def __init__(self, text_data: _Optional[str] = ...) -> None: ...

class Types(_message.Message):
    __slots__ = ["bool_t", "double_t", "float_t", "signed_integer_32", "signed_integer_64", "unsigned_integer_32", "unsigned_integer_64"]
    BOOL_T_FIELD_NUMBER: _ClassVar[int]
    DOUBLE_T_FIELD_NUMBER: _ClassVar[int]
    FLOAT_T_FIELD_NUMBER: _ClassVar[int]
    SIGNED_INTEGER_32_FIELD_NUMBER: _ClassVar[int]
    SIGNED_INTEGER_64_FIELD_NUMBER: _ClassVar[int]
    UNSIGNED_INTEGER_32_FIELD_NUMBER: _ClassVar[int]
    UNSIGNED_INTEGER_64_FIELD_NUMBER: _ClassVar[int]
    bool_t: bool
    double_t: float
    float_t: float
    signed_integer_32: int
    signed_integer_64: int
    unsigned_integer_32: int
    unsigned_integer_64: int
    def __init__(self, unsigned_integer_32: _Optional[int] = ..., signed_integer_32: _Optional[int] = ..., bool_t: bool = ..., float_t: _Optional[float] = ..., double_t: _Optional[float] = ..., unsigned_integer_64: _Optional[int] = ..., signed_integer_64: _Optional[int] = ...) -> None: ...

class Example_Enum(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
