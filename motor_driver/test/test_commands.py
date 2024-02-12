import pytest

from motor_driver.generated import commands_pb2

# def test_print_message_size():
#     print("Intermediate printed messages\n")
#     print("Run tests with command:\n colcon test --packages-select motor_driver --event-handlers console_cohesion+")
#     config = commands_pb2.Config_Data()
#     config.address = 130
#     message = commands_pb2.Serial_Message()
#     message.opcode = commands_pb2.CONFIG_MOTOR
#     message.configData.address = 130
#     print(message.ByteSize())
#     assert 1 == 1

# def test_opcode_only():
#     message = commands_pb2.Serial_Message()
#     message.opcode = commands_pb2.STOP_ALL_MOTORS
#     assert message.WhichOneof("data") is None

#     message_1 = commands_pb2.Serial_Message()
#     message_1.opcode = commands_pb2.STOP_ALL_MOTORS
#     message_1.configData.motorID = 3
#     assert message.ByteSize() < message_1.ByteSize()

#     # Serialize and Deserialize
#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.STOP_ALL_MOTORS
#     assert deserialized_message.WhichOneof("data") is None

# def test_config_data():
#     message = commands_pb2.Serial_Message()
    
#     # Motor ID test
#     message.opcode = commands_pb2.CONFIG_MOTOR
#     message.configData.motorID = 1
#     assert message.WhichOneof("data") == "configData"
#     assert message.configData.WhichOneof("values") == "motorID"

#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.CONFIG_MOTOR
#     assert deserialized_message.WhichOneof("data") == "configData"
#     assert deserialized_message.configData.WhichOneof("values") == "motorID"
#     assert deserialized_message.configData.motorID == 1

#     # Serial Pin test
#     message.opcode = commands_pb2.CONFIG_MOTOR
#     message.configData.serialPin = 1024
#     assert message.WhichOneof("data") == "configData"
#     assert message.configData.WhichOneof("values") == "serialPin"

#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.CONFIG_MOTOR
#     assert deserialized_message.WhichOneof("data") == "configData"
#     assert deserialized_message.configData.WhichOneof("values") == "serialPin"
#     assert deserialized_message.configData.serialPin == 1024

#     # Motor Num test
#     message.opcode = commands_pb2.CONFIG_MOTOR
#     message.configData.motorNum = 2
#     assert message.WhichOneof("data") == "configData"
#     assert message.configData.WhichOneof("values") == "motorNum"

#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.CONFIG_MOTOR
#     assert deserialized_message.WhichOneof("data") == "configData"
#     assert deserialized_message.configData.WhichOneof("values") == "motorNum"
#     assert deserialized_message.configData.motorNum == 2

#     # address test
#     message.opcode = commands_pb2.CONFIG_MOTOR
#     message.configData.address = 192061
#     assert message.WhichOneof("data") == "configData"
#     assert message.configData.WhichOneof("values") == "address"

#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.CONFIG_MOTOR
#     assert deserialized_message.WhichOneof("data") == "configData"
#     assert deserialized_message.configData.WhichOneof("values") == "address"
#     assert deserialized_message.configData.address == 192061

#     # inverted test
#     message.opcode = commands_pb2.CONFIG_MOTOR
#     message.configData.inverted = True
#     assert message.WhichOneof("data") == "configData"
#     assert message.configData.WhichOneof("values") == "inverted"

#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.CONFIG_MOTOR
#     assert deserialized_message.WhichOneof("data") == "configData"
#     assert deserialized_message.configData.WhichOneof("values") == "inverted"
#     assert deserialized_message.configData.inverted == True

# def test_turn_motor():
#     message = commands_pb2.Serial_Message()
#     message.opcode = commands_pb2.TURN_MOTOR
#     message.motorCommand.percentOutput = 0.08
#     assert message.WhichOneof("data") == "motorCommand"

#     # Serialize and Deserialize
#     serialized_message = message.SerializeToString()
#     assert message.ByteSize() == len(serialized_message)
#     deserialized_message = commands_pb2.Serial_Message()
#     deserialized_message.ParseFromString(serialized_message)

#     assert deserialized_message.opcode == commands_pb2.TURN_MOTOR
#     assert deserialized_message.WhichOneof("data") == "motorCommand"
#     # assert deserialized_message.motorCommand.percentOutput == 0.08 
#     # account for floating point imprecision