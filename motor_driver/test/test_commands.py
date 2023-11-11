import pytest

from motor_driver.generated import commands_pb2

def test_print_message_size():
    print("Intermediate printed messages\n")
    print("Run tests with command:\n colcon test --packages-select motor_driver --event-handlers console_cohesion+")
    config = commands_pb2.Config_Data()
    config.address = 130
    message = commands_pb2.Serial_Message()
    message.opcode = commands_pb2.CONFIG_MOTOR
    message.config_data.address = 130
    print(message.ByteSize())
    assert 1 == 1