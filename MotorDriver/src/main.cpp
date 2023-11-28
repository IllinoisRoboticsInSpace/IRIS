#include "Sabertooth.h"
#include "MotorDriver.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

#define DEFAULT_SABERTOOTH_BAUD_RATE 9600 // 9600 is the default baud rate for Sabertooth packet serial.
#define DEFAULT_SABERTOOTH_PORT 128 // 128 is the default port (for some reason?)

Sabertooth ST(DEFAULT_SABERTOOTH_PORT);
MotorDriver driver;

void setup() {
  //? below block in the init_motor_driver_function? or the constructor itself in MotorDriver
  driver = MotorDriver(DEFAULT_SERIAL_TRANSFER_BAUD_RATE, std::array<MotorDriver::MotorDriverConfig, NUM_ARDUINO_PINS>(), &ST);
  SabertoothTXPinSerial.begin(DEFAULT_SABERTOOTH_BAUD_RATE);
  Serial.begin(DEFAULT_SERIAL_TRANSFER_BAUD_RATE); //! DUPLICATION: also in motordriver class, original comment was "For printing"
  ST.autobaud();

  // while (driver.init_motor_driver() == false)
  if (driver.initMotorDriver() == false)
  {
    Serial.println("init failed");
  }
}

void loop() {
  driver.update();
}