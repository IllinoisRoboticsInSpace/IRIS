#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothConfig, MAX_MOTOR_CONFIGS> configs)
    : serialTransferBaudRate(serialTransferBaudRate), configs(configs), initialized(false)
{
    
}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
{
    MotorDriver(serialTransferBaudRate, std::array<SabertoothConfig, MAX_MOTOR_CONFIGS>());
}

MotorDriver::MotorDriver()
{
    MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE, std::array<SabertoothConfig, MAX_MOTOR_CONFIGS>());
}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    Serial.begin(serialTransferBaudRate);
    Serial.println("Motor Driver Initialized");
    initialized = true;
    return true;
}

SabertoothConfig MotorDriver::getConfig(unsigned int motorID)
{
    return configs[motorID];
}

void MotorDriver::setConfig(unsigned int motorID, SabertoothConfig config)
{
    configs[motorID] = config;
}

void MotorDriver::read()
{
    
}

void MotorDriver::parse()
{ 

}

void MotorDriver::execute()
{

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
    // read encoder data
    // send back encoder data
    // run PID loops
    read();
    parse();
    execute();
}
