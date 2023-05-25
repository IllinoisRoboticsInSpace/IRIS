#include <Arduino.h>
#include "Sabertooth.h"
#include "MotorDriver.h"

Sabertooth ST(128);

void setup() {
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  Serial.begin(112500);//For printing
  ST.autobaud();
}

void loop() {
  delay(1000);
}