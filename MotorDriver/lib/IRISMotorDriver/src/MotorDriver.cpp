#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver()
{
    //Apply default config for default constructor
    MotorDriver(0, SERIAL_DEFAULT_BAUD_RATE);
}

MotorDriver::MotorDriver(Sabertooth *st, unsigned int baudRate)
{
    this->st = st;
    initialized = false;
    config.baudRate = baudRate;
    config.mode_auto = true;
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
bool MotorDriver::init_motor_driver()
{
    if (st == 0)
    {
        return false;
    }
    Serial.begin(config.baudRate); //! DUPLICATION: also in main.cpp
    Serial.println("Motor Driver Initialized");
    initialized = true;
    return true;
}

void MotorDriver::setMode(bool mode_auto)
{
    this->config.mode_auto = mode_auto;
}

void MotorDriver::setBaudRate(unsigned int baudRate)
{
    config.baudRate = baudRate;
    if (initialized) {
        initialized = false;
        Serial.end();
        Serial.begin(config.baudRate);
        Serial.println("baud rate changed");
        initialized = true;
    }
}