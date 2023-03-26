#ifndef _DEBUG_UTILS_
#define _DEBUG_UTILS_

#include <stdio.h>

#define MAX_DEBUG_MESSAGE 128
static char debug_buffer[MAX_DEBUG_MESSAGE];

#if(DEBUG == true)
#define PRINTF_SERIAL_DEBUG(format, ...)        \
  sprintf(debug_buffer, format, __VA_ARGS__);   \
  Serial.print(debug_buffer);
#else
#define PRINTF_SERIAL_DEBUG(format, ...)
#endif

/* Initialize all debug options */
void setup_debug();
/* Setup serial_debugging */
void setup_serial_debug();
/* Initialize the built in led pinmode */
void setup_led_debugging();
/* Immediately turn on led */
void debug_led_on();
/* Immediately turn off led */
void debug_led_off();

#endif /* _DEBUG_UTILS_ */