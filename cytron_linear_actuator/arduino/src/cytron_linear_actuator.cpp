#include "CytronMotorDriver.h"
#include "Arduino.h"

// Define the motor driver mode
#define PWM_PIN 9 // PWM pin = 9
#define DIR_PIN 4// DIR pin = 10
#define BAUD_RATE 9600

// Configure the motor driver
CytronMD motor(PWM_DIR, PWM_PIN, DIR_PIN);

// Serial communication variables
char inChar;
String currentCommand = "";

void setup() {
  Serial.begin(BAUD_RATE);
  motor.setSpeed(0);
  
  Serial.println("Cytron Linear Actuator Controller Ready");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    inChar = (char)Serial.read();
    if (inChar == 'M') {
      currentCommand = "";
    }
    else if (isDigit(inChar) || inChar == '-') {
      currentCommand += inChar;
    }
    else if (inChar == '\n') {
      // Process the command
      if (currentCommand.length() > 0) {
        int speed = currentCommand.toInt();
        
        // Ensure speed is within valid range (-255 to 255)
        speed = constrain(speed, -255, 255);
        
        if (speed == 0) {
          motor.setSpeed(0);
          Serial.println("Linear Actuator Stopped");
          currentCommand = "";
          return;
        } else {
        // Set the motor speed
        motor.setSpeed(speed);
        Serial.print("Linear Actuator Speed set to: ");
        Serial.println(speed);
        currentCommand = "";
        }
      }
    }
  }
}
