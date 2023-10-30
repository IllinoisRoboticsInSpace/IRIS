#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver()
{
    //Apply default config for default constructor
    MotorDriver(SERIAL_DEFAULT_BAUD_RATE);
}

MotorDriver::MotorDriver(unsigned int baudRate)
{
    initialized = false;
    config.baudRate = baudRate;
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
    Serial.begin(config.baudRate); //! DUPLICATION: also in main.cpp
    Serial.println("Motor Driver Initialized");
    initialized = true;
}