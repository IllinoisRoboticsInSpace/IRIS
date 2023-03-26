#include "debug_utils.h"

/* Initialize all debug options */
void setup_debug()
{
    setup_led_debugging();
    setup_serial_debug();
}
/* Setup serial_debugging */
void setup_serial_debug()
{
    Serial.begin(9600);
}

/* Setup the arduino for led debugging */
void setup_led_debugging()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

/* Immediately turn on led */
void debug_led_on()
{
    digitalWrite(LED_BUILTIN, HIGH);
}

/* Immediately turn off led */
void debug_led_off()
{
    digitalWrite(LED_BUILTIN, LOW);
}