#include "Sabertooth.h"
#include "MotorDriver.h"
#include "CytronMotorDriver.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.


// The setup routine runs once when you press reset.
void setup() {
  
}


// The loop routine runs over and over again forever.
void loop() {
  motor.setSpeed(128);  // Run forward at 50% speed.
  delay(1000);
  
  motor.setSpeed(255);  // Run forward at full speed.
  delay(1000);

  motor.setSpeed(0);    // Stop.
  delay(1000);

  motor.setSpeed(-128);  // Run backward at 50% speed.
  delay(1000);
  
  motor.setSpeed(-255);  // Run backward at full speed.
  delay(1000);

  motor.setSpeed(0);    // Stop.
  delay(1000);
}
//Sabertooth ST(128);

// MotorDriver driver;

// void setup() {
//   SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
//   Serial.begin(112500);//For printing
//   ST.autobaud();

//   driver.init_motor_driver();
// }

// void loop() {
//   driver.update();
// }