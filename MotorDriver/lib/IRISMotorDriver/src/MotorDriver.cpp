#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver()
{
    //Apply default config for default constructor
}

/**
 * Run motor driver update loop
*/
void MotorDriver::update()
{
    //Read serial data

    //parse serial data

    //execute command




    //FUTURE:
    //read encoder data

    //send back encoder data

    //run PID loops
}

/**
 * Initialize motor driver communication lines and supporting devices
*/
void MotorDriver::init_motor_driver()
{
    Serial.begin(112500);
    Serial.println("Motor Driver Initialized");
}