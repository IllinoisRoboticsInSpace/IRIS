# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from motor_driver.generated import example_pb2

def test_example():
    print("Intermediate printed messages\n")
    print("Run tests with command:\n colcon test --packages-select motor_driver --event-handlers console_cohesion+")
    assert 1 == 1

def test_types_message():
    message = example_pb2.Types()
    message.signed_integer_32 = -10
    message.unsigned_integer_32 = 10
    message.bool_t = True
    message.float_t = 10.5
    message.double_t = -10.5
    message.unsigned_integer_64 = 100
    message.signed_integer_64 = -100


    assert -10 == message.signed_integer_32, f"int32 should be -10"
    assert 10 == message.unsigned_integer_32, f"uint32 should be 10"
    assert True == message.bool_t, f"bool should be true"
    assert 10.5 == message.float_t, f"float should be 10.5"
    assert -10.5 == message.double_t, f"double should be -10.5"
    assert 100 == message.unsigned_integer_64, f"uint64 should be 100"
    assert -100 == message.signed_integer_64, f"int64 should be -100"

    # Serialize & Deserialize
    serialized_message = message.SerializeToString()
    assert message.ByteSize() == len(serialized_message)
    deserialized_message = example_pb2.Types()
    deserialized_message.ParseFromString(serialized_message)

    assert -10 == deserialized_message.signed_integer_32, f"int32 should be -10"
    assert 10 == deserialized_message.unsigned_integer_32, f"uint32 should be 10"
    assert True == deserialized_message.bool_t, f"bool should be true"
    assert 10.5 == deserialized_message.float_t, f"float should be 10.5"
    assert -10.5 == deserialized_message.double_t, f"double should be -10.5"
    assert 100 == deserialized_message.unsigned_integer_64, f"uint64 should be 100"
    assert -100 == deserialized_message.signed_integer_64, f"int64 should be -100"

    
def test_nested_message():
    message = example_pb2.Nested()
    nested_message = example_pb2.A()
    nested_message.a = 10
    message.nested_a.CopyFrom(nested_message)
    assert 10 == message.nested_a.a

    message.nested_a.a = 2
    assert 2 == message.nested_a.a

    # Serialize & Deserialize
    serialized_message = message.SerializeToString()
    assert message.ByteSize() == len(serialized_message)
    deserialized_message = example_pb2.Nested()
    deserialized_message.ParseFromString(serialized_message)
    
    assert 2 == deserialized_message.nested_a.a


def test_repeated_message():
    pass

def test_string_message():
    message = example_pb2.Text()
    str = "String Message Test"
    message.text_data = str
    assert str == message.text_data
    
    #Serialize and Deserialize
    serialized_message = message.SerializeToString()
    assert message.ByteSize() == len(serialized_message)
    deserialized_message = example_pb2.Text()
    deserialized_message.ParseFromString(serialized_message)

    assert str == deserialized_message.text_data

def test_bytes_message():
    message = example_pb2.Raw_Bytes()
    message.b = bytes([0,1,2])
    assert 0 == message.b[0]
    assert 1 == message.b[1]
    assert 2 == message.b[2]

    serialized_message = message.SerializeToString()
    assert message.ByteSize() == len(serialized_message)
    deserialized_message = example_pb2.Raw_Bytes()
    deserialized_message.ParseFromString(serialized_message)

    assert 0 == deserialized_message.b[0]
    assert 1 == deserialized_message.b[1]
    assert 2 == deserialized_message.b[2]


def test_one_of_message():
    # Check that type selection is working
    message = example_pb2.Oneof()
    assert message.WhichOneof("Union") is None

    message.bool_choice = True
    assert message.WhichOneof("Union") == "bool_choice"
    bool_message_size = message.ByteSize()

    message.int32_choice = 10
    assert message.WhichOneof("Union") == "int32_choice"
    int32_message_size = message.ByteSize()

    message.double_choice = 10.5
    assert message.WhichOneof("Union") == "double_choice"
    double_message_size = message.ByteSize()

    assert int32_message_size < double_message_size
    assert bool_message_size <= int32_message_size

    # Serialize and Deserialize
    serialized_message = message.SerializeToString()
    assert message.ByteSize() == len(serialized_message)
    deserialized_message = example_pb2.Oneof()
    deserialized_message.ParseFromString(serialized_message)

    assert message.WhichOneof("Union") == "double_choice"
    assert 10.5 == message.double_choice


def test_optional_message():
    message = example_pb2.Optional()
    message.option = 10
    set_size = message.ByteSize()
    
    message.Clear()
    cleared_size = message.ByteSize()

    assert cleared_size < set_size
    assert message.HasField("option") == False

    # Serialize and Deserialize
    message.option = 10
    serialized_message = message.SerializeToString()
    assert message.ByteSize() == len(serialized_message)
    deserialized_message = example_pb2.Optional()
    deserialized_message.ParseFromString(serialized_message)

    assert message.option == deserialized_message.option