#include "DebugTools.h"
#include "MotorDriver.h"
#include "generated/commands.h"

#if ENABLE_DEBUG == true
char debug_buffer[DEBUG_BUFFER_SIZE];
EmbeddedProto::string_view debug_buffer_string_view = {.data = debug_buffer, .size = DEBUG_BUFFER_SIZE};
Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES> serial_container;
Debug_Message<MAX_DEBUG_STRING_SIZE_BYTES> debug_message;

// IMPORTANT NOTE:
// The platformio serial monitor replaces non ASCII
// characters with EFBFBD which is the End of File (EOF)
// character sequence. This means that when debugging using
// the platformio serial monitor you will see different chars
// and number of chars printed compared to what was actually
// put on the serial line.
// https://stackoverflow.com/a/6544206

void print_message_serialized(int message_length)
{
    // TODO: figure out a way to only set message once
    serial_container.set_opcode(Opcode_To_Jetson::DEBUG_MESSAGE);
    serial_container.set_debug_message(debug_message);

    char save_null_char_element;
    for (int offset = 0; offset < message_length; offset += MAX_DEBUG_STRING_SIZE_BYTES)
    {
        int chars_left = message_length - offset;
        int chars_to_print = MAX_DEBUG_STRING_SIZE_BYTES;
        if (chars_left < MAX_DEBUG_STRING_SIZE_BYTES)
        {
            chars_to_print = chars_left;
        }
        save_null_char_element = debug_buffer[offset + chars_to_print];
        debug_buffer[offset + chars_to_print] = '\0';

        char * start_prt = debug_buffer + offset;
        debug_message.mutable_debug_data() = start_prt;
        serial_container.set_debug_message(debug_message);
        MotorDriver::send_message(serial_container);

        // Revert change to null char
        debug_buffer[offset + chars_to_print] = save_null_char_element;
    }
}

void print_message_serialized_constant_string(char buffer[], int message_length)
{
    // TODO: figure out a way to only set message once
    serial_container.set_opcode(Opcode_To_Jetson::DEBUG_MESSAGE);
    serial_container.set_debug_message(debug_message);

    // Set ending null character for full debug string printing
    debug_buffer[MAX_DEBUG_STRING_SIZE_BYTES] = '\0';
    
    for (int offset = 0; offset < message_length; offset += MAX_DEBUG_STRING_SIZE_BYTES)
    {
        // Might be optimizable further because you can precompute the
        // offset that this condition might be true for.
        int chars_left = message_length - offset;
        char * start_prt = buffer + offset;
        if (chars_left < MAX_DEBUG_STRING_SIZE_BYTES)
        {
            // In this case there is always space to place a null character at the end.
            memcpy(debug_buffer, start_prt, chars_left);
            debug_buffer[chars_left] = '\0';
        }
        else
        {
            memcpy(debug_buffer, start_prt, MAX_DEBUG_STRING_SIZE_BYTES);
        }
        
        debug_message.mutable_debug_data() = debug_buffer;
        serial_container.set_debug_message(debug_message);
        MotorDriver::send_message(serial_container);
    }
}
#endif