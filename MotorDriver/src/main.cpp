#include "generated/commands.h"
#include "Sabertooth.h"
#include "MotorDriver.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

Sabertooth ST(128);

void setup() {
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  Serial.begin(112500);//For printing
  ST.autobaud();
  init_motor_driver();
}

void loop() {
  delay(1000);
}