#ifndef _DEBUG_TOOLS_
#define _DEBUG_TOOLS_

#include <Arduino.h>

// Do not commit with ENABLE_DEBUG set to true
#define ENABLE_DEBUG false

#if ENABLE_DEBUG == true

    #ifndef DEBUG_BUFFER_SIZE
        #define DEBUG_BUFFER_SIZE 256
        extern char debug_buffer[DEBUG_BUFFER_SIZE];
    #endif

    #define DEBUG_PRINT(string, ...)              \
    {                                             \
    sprintf(debug_buffer, string, ##__VA_ARGS__); \
    Serial.println(debug_buffer);                 \
    }
#else
    #define DEBUG_PRINT(string, ...) {}
#endif

#endif