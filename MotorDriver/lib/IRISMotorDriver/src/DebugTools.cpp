#include "DebugTools.h"
#include "MotorDriver.h"

#if ENABLE_DEBUG == true
char debug_buffer[DEBUG_BUFFER_SIZE];
EmbeddedProto::string_view debug_buffer_string_view = {.data = debug_buffer, .size = DEBUG_BUFFER_SIZE};
#endif