#include "Sabertooth.h"
#include "MotorDriver.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

#define ST_DEFAULT_BAUD_RATE 9600 // 9600 is the default baud rate for Sabertooth packet serial.
#define ST_DEFAULT_PORT 128 // 128 is the default port (for some reason?)

Sabertooth ST(ST_DEFAULT_PORT);
MotorDriver driver;

void setup() {
  driver = MotorDriver(&ST, SERIAL_DEFAULT_BAUD_RATE);
  SabertoothTXPinSerial.begin(ST_DEFAULT_BAUD_RATE);
  Serial.begin(SERIAL_DEFAULT_BAUD_RATE); //! DUPLICATION: also in motordriver class, original comment was "For printing"
  ST.autobaud();

  // while (driver.init_motor_driver() == false)
  if (driver.init_motor_driver() == false)
  {
    Serial.println("init failed");
  }
}

void loop() {
  driver.update();
}