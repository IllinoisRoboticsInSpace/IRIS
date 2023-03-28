#ifndef _MOTOR_PIN_CONFIG_
#define _MOTOR_PIN_CONFIG_

#include <Arduino.h>

// A config for the arduino due
// Available addresses for packetized serial mode are 128-135
// DIP switch address config: Description from http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm:

// Not used but might be used in the future
// typedef struct SabertoothConfig{
//   int address;
//   USARTClass serial;
//   MotorConfig S1Motor;
//   MotorConfig S2Motor;
// } SabertoothConfig;
//{.c = {1000, NULL}, .f = 42.0, .s = "foo"};
// SabertoothConfig drive_sabertooth_config = {address = 1, serial = Serial1
//                                             S1Motor = {}
//                                             S2Motor = {}}

// Not used but might be used in the future
// typedef struct MotorConfig{
  
// } MotorConfig;

// NOTE: Hardware Serial TX0 interferes with the usb serial output

// Sabertooth 1
// Motors (1,2): Right Back Bucket Ladder Motor, Drive Right
#define DRIVE_SABER_ADDRESS 128        // DIP switch address
#define DRIVE_SABER_SERIAL Serial1     // Hardware Serial TX1
#define DRIVE_LEFT_MOTOR 1             // S1
#define DRIVE_RIGHT_MOTOR 2            // S2

// Sabertooth 2
// Motors (1,2): Excavator Internal Motor, Excavator Threaded Rod Actuator
#define S2_SABER_ADDRESS 129           // DIP switch address
#define S2_SABER_SERIAL Serial2        // Hardware Serial TX2
#define S2_LEFT_MOTOR 1                // S1
#define S2_RIGHT_MOTOR 2               // S2

// Sabertooth 3
// Motors (1,2): Left Collection Motor, Left Drive Motor
#define S3_SABER_ADDRESS 130            // DIP switch address
#define S3_SABER_SERIAL Serial3         // Hardware Serial TX3
#define S3_LEFT_MOTOR 1                 // S1
#define S3_RIGHT_MOTOR 2                // S2

// 10 Amp Board
// Motors: Excavator Pivoting Linear Actuator
#define A_DIRECTION 22
#define A_PWM 2

#endif /* _MOTOR_PIN_CONFIG_ */
