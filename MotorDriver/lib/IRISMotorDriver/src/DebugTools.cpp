#include "DebugTools.h"
#include "MotorDriver.h"
#include "generated/commands.h"

#if ENABLE_DEBUG == true
char debug_buffer[DEBUG_BUFFER_SIZE];
EmbeddedProto::string_view debug_buffer_string_view = {.data = debug_buffer, .size = DEBUG_BUFFER_SIZE};
Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES> serial_container;
Debug_Message<MAX_DEBUG_STRING_SIZE_BYTES> debug_message;

void print_message_serialized(int message_length)
{
    // TODO: figure out a way to only set message once
    serial_container.set_opcode(Opcode_To_Jetson::DEBUG_MESSAGE);
    serial_container.set_debug_message(debug_message);

    for (int offset = 0; offset < message_length; offset += MAX_DEBUG_STRING_SIZE_BYTES)
    {
        debug_message.mutable_debug_data() = debug_buffer + offset;
        MotorDriver::send_message(serial_container);
    }
}

void print_message_serialized_constant_string(char buffer[], int message_length)
{
    // TODO: figure out a way to only set message once
    serial_container.set_opcode(Opcode_To_Jetson::DEBUG_MESSAGE);
    serial_container.set_debug_message(debug_message);

    for (int offset = 0; offset < message_length; offset += MAX_DEBUG_STRING_SIZE_BYTES)
    {
        debug_message.mutable_debug_data() = buffer + offset;
        MotorDriver::send_message(serial_container);
    }
}
#endif