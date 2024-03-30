#ifndef _DEBUG_TOOLS_
#define _DEBUG_TOOLS_

#include "MotorDriver.h"

#include <Arduino.h>
#include <Defines.h>

// Using debug tools with motor driver without
// debug mode will cause unintended behavior.
// Must start usb serial transfer by running:
// Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
// Do not commit with ENABLE_DEBUG set to true
#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG true
#endif

// Option to transfer data inside serialized
// protobuf debug message.
// This option is slower than not serializing
// ,but the host's python driver will be able
// deserialize the data and print it out, while
// the arduino is sending other proto messages.
#define SERIALIZE_DEBUG_MESSAGES false

#if ENABLE_DEBUG == true

    #ifndef DEBUG_BUFFER_SIZE
        #define DEBUG_BUFFER_SIZE 256
        extern char debug_buffer[DEBUG_BUFFER_SIZE];
        extern EmbeddedProto::string_view debug_buffer_string_view;
        extern Debug_Message<MAX_DEBUG_STRING_SIZE_BYTES> debug_message;
    #endif


    // Do not count null character for computing character to print
    #if SERIALIZE_DEBUG_MESSAGES == true
        void print_message_serialized(int message_length);
        void print_message_serialized_constant_string(char constant_buffer[], int message_length);

        #define DEBUG_PRINTF(string, ...)                                       \
        {                                                                       \
        print_message_serialized(sprintf(debug_buffer, string, ##__VA_ARGS__)); \
        }

        //Checks if environment variable has been set
        #ifdef MSG_TO_STRING
        #define DEBUG_PRINT_MESSAGE(proto)                                              \
        {                                                                               \
        EmbeddedProto::string_view view = proto.to_string(debug_buffer_string_view);    \
        print_message_serialized(DEBUG_BUFFER_SIZE - view.size);                        \
        }
        #else
        #define DEBUG_PRINT_MESSAGE(proto)              \
        {                                               \
        DEBUG_PRINTLN("MSG_TO_STRING not defined");     \
        }
        #endif
        // How to get at compile time a string's length
        // https://devblogs.microsoft.com/oldnewthing/20221114-00/?p=107393
        // https://stackoverflow.com/a/26082447

        template< int N >
        constexpr int length( char const (&)[N] )
        {
            return N-1;
        }

        #define DEBUG_PRINT(string)                                         \
        {                                                                   \
        char message[] = string;                                            \
        print_message_serialized_constant_string(message, length(string));  \
        }

        // TODO: Figure out how to get newline to be inserted
        // at end of string literal at compile time

        #define DEBUG_PRINTLN(string)                                       \
        {                                                                   \
        char message[] = string;                                            \
        print_message_serialized_constant_string(message, length(string));  \
        debug_buffer[0] = '\n';                                             \
        print_message_serialized(1);                                        \
        }
    #else
        #define DEBUG_PRINTF(string, ...)               \
        {                                               \
        sprintf(debug_buffer, string, ##__VA_ARGS__);   \
        Serial.print(debug_buffer);                     \
        }

        //Checks if environment variable has been set
        #ifdef MSG_TO_STRING
        #define DEBUG_PRINT_MESSAGE(proto)              \
        {                                               \
        proto.to_string(debug_buffer_string_view);      \
        Serial.print(debug_buffer_string_view.data);    \
        }
        #else
        #define DEBUG_PRINT_MESSAGE(proto)              \
        {                                               \
        DEBUG_PRINTLN("MSG_TO_STRING not defined");     \
        }
        #endif

        #define DEBUG_PRINT(string)                     \
        {                                               \
        Serial.print(string);                           \
        }

        #define DEBUG_PRINTLN(string)                   \
        {                                               \
        Serial.println(string);                         \
        }
    #endif
#else
    #define DEBUG_PRINTF(string, ...) {}
    #define DEBUG_PRINT_MESSAGE(proto) {}
    #define DEBUG_PRINT(string) {}
    #define DEBUG_PRINTLN(string) {}
#endif

// Resource on counting number of input arguments in variadic macro:
// https://stackoverflow.com/questions/2124339/c-preprocessor-va-args-number-of-arguments

#endif