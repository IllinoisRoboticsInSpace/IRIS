#include "Sabertooth.h"
#include "MotorDriver.h"
#include "DebugTools.h"
#include "generated/commands.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

#define DEFAULT_SABERTOOTH_BAUD_RATE 9600 // 9600 is the default baud rate for Sabertooth packet serial.

MotorDriver driver;

// Serial: line from us to arduino
// Serial1: line from arduino to sabertooth

void setup() {
  Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);

  driver = MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE);

  if (driver.initMotorDriver() == false)
  {
    DEBUG_PRINTLN("Initialization Failed");
  }
}

void loop() {
  driver.update();
}