#include "CytronMotorDriver.h"
#include "Arduino.h"


// Configure the motor driver.
CytronMD motor(PWM_DIR, 9, 10);  // PWM = Pin 3, DIR = Pin 4.


// The setup routine runs once when you press reset.
void setup() {
  
}


// The loop routine runs over and over again forever.
void loop() {
  motor.setSpeed(128);  // Run forward at 50% speed.
  delay(15000);
  
  motor.setSpeed(255);  // Run forward at full speed.
  delay(15000);

  motor.setSpeed(0);    // Stop.
  delay(15000);

  motor.setSpeed(-128);  // Run backward at 50% speed.
  delay(10000);
  
  motor.setSpeed(-255);  // Run backward at full speed.
  delay(10000);

  motor.setSpeed(0);    // Stop.
  delay(1000);
}