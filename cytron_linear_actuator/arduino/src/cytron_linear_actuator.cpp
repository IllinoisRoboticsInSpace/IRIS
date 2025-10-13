#include "CytronMotorDriver.h"
#include "Arduino.h"

// Define the motor driver mode
#define PWM_PIN 9 // PWM pin = 9
#define DIR_PIN 4// DIR pin = 10
#define BAUD_RATE 9600

// Configure the motor driver
CytronMD motor(PWM_DIR, PWM_PIN, DIR_PIN);

// SerialUSB communication variables
char inChar;
String currentCommand = "";
int numSetup = 0;

void setup() {
  SerialUSB.begin(BAUD_RATE);
  motor.setSpeed(0);
  SerialUSB.println("Cytron Linear Actuator Controller Ready");
  numSetup++;
}

void loop() {
  // Check for incoming serial data
  if (SerialUSB.available() > 0) {
    SerialUSB.println("Data Received");
    inChar = (char)SerialUSB.read();
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
          SerialUSB.println("Linear Actuator Stopped");
          currentCommand = "";
          return;
        } else {
        // Set the motor speed
        motor.setSpeed(speed);
        SerialUSB.print("Linear Actuator Speed set to: ");
        SerialUSB.println(speed);
        currentCommand = "";
        }
      }
    }
    SerialUSB.println("NumSetup " + numSetup);
  }
}
