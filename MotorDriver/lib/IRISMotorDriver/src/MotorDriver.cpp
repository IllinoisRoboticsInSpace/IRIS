#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<MotorDriver::MotorDriverConfig, MAX_MOTOR_ID + 1> configs)
{
    this->initialized = false;
    this->serialTransferBaudRate = serialTransferBaudRate;
    this->configs = configs;
}

MotorDriver::MotorDriver()
{
    MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE, std::array<MotorDriver::MotorDriverConfig, MAX_MOTOR_ID + 1>());
}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    Serial.begin(serialTransferBaudRate); //! DUPLICATION: also in main.cpp
    Serial.println("Motor Driver Initialized");
    initialized = true;
    return true;
}

bool MotorDriver::getInitialized()
{
    return initialized;
}

unsigned int MotorDriver::getSerialTransferBaudRate()
{
    return serialTransferBaudRate;
}

void MotorDriver::setSerialTransferBaudRate(unsigned int serialTransferBaudRate)
{
    this->serialTransferBaudRate = serialTransferBaudRate;

    if (initialized) {
        initialized = false;
        Serial.end();
        Serial.begin(serialTransferBaudRate);
        Serial.println("baud rate changed");
        initialized = true;
    }
}

std::array<MotorDriver::MotorDriverConfig, MAX_MOTOR_ID + 1> MotorDriver::getConfigs()
{
    return configs;
}

void MotorDriver::setConfigs(std::array<MotorDriver::MotorDriverConfig, MAX_MOTOR_ID + 1> configs)
{
    this->configs = configs;
}

void MotorDriver::read(){
    
}

void MotorDriver::parse(){ 

}

void MotorDriver::execute(){

}

/**
 * Run motor driver update loop
*/
void MotorDriver::update()
{
    if (!initialized)
    {
        return;
    }

    //FUTURE:
    //read encoder data
    //send back encoder data
    //run PID loops
    read();
    parse();
    execute();
}
