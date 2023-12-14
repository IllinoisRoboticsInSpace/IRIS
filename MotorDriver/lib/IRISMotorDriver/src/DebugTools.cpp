#include "DebugTools.h"
#include "MotorDriver.h"

#if ENABLE_DEBUG == true
char debug_buffer[DEBUG_BUFFER_SIZE];
Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
#endif