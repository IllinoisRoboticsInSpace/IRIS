#ifndef _DEBUG_TOOLS_
#define _DEBUG_TOOLS_

#include <Arduino.h>

// Using debug tools with motor driver without
// debug mode will cause unintended behavior.
// Must start usb serial transfer by running:
// Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
// Do not commit with ENABLE_DEBUG set to true
#define ENABLE_DEBUG false

#if ENABLE_DEBUG == true

    #ifndef DEBUG_BUFFER_SIZE
        #define DEBUG_BUFFER_SIZE 256
        extern char debug_buffer[DEBUG_BUFFER_SIZE];
    #endif

    #define DEBUG_PRINTF(string, ...)               \
    {                                               \
    sprintf(debug_buffer, string, ##__VA_ARGS__);   \
    Serial.println(debug_buffer);                   \
    }

    #define DEBUG_PRINT(string)                     \
    {                                               \
    Serial.println(string);                         \
    }

#else
    #define DEBUG_PRINTF(string, ...) {}
    #define DEBUG_PRINT(string) {}
#endif

// Resource on counting number of input arguments in variadic macro:
// https://stackoverflow.com/questions/2124339/c-preprocessor-va-args-number-of-arguments

#endif