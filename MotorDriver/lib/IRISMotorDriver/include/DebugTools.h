#ifndef _DEBUG_TOOLS_
#define _DEBUG_TOOLS_

#define ENABLE_DEBUG true

#if ENABLE_DEBUG == true

    #ifndef DEBUG_BUFFER_SIZE
    #define DEBUG_BUFFER_SIZE 256
    char debug_buffer[DEBUG_BUFFER_SIZE];
    #endif

    #define DEBUG_PRINT(string, ...)                \
    {                                               \
    sprintf(debug_buffer, string, ##__VA_ARGS__); \
    Serial.println(debug_buffer);                 \
    }
#else
    #define DEBUG_PRINT(string, ...) {}
#endif

#endif