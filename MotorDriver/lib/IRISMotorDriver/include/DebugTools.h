#ifndef _DEBUG_TOOLS_
#define _DEBUG_TOOLS_

#include <Arduino.h>
#include <Defines.h>

// Using debug tools with motor driver without
// debug mode will cause unintended behavior.
// Must start usb serial transfer by running:
// Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
// Do not commit with ENABLE_DEBUG set to true
#define ENABLE_DEBUG true

#if ENABLE_DEBUG == true

    #ifndef DEBUG_BUFFER_SIZE
        #define DEBUG_BUFFER_SIZE 256
        extern char debug_buffer[DEBUG_BUFFER_SIZE];
        extern EmbeddedProto::string_view debug_buffer_string_view;
    #endif

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
    Serial.print("MSG_TO_STRING not defined");      \
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

#else
    #define DEBUG_PRINTF(string, ...) {}
    #define DEBUG_PRINT_MESSAGE(proto) {}
    #define DEBUG_PRINT(string) {}
    #define DEBUG_PRINTLN(string) {}
#endif

// Resource on counting number of input arguments in variadic macro:
// https://stackoverflow.com/questions/2124339/c-preprocessor-va-args-number-of-arguments

#endif