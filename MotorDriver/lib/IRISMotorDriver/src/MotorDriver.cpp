#include "MotorDriver.h"
#include "DebugTools.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> configs)
    : serialTransferBaudRate(serialTransferBaudRate), configs(configs)
{

}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
{
    MotorDriver(serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>());
}

MotorDriver::MotorDriver()
{
    MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>());
}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    Serial.begin(serialTransferBaudRate);
    return true;
}

SabertoothOperator MotorDriver::getConfig(unsigned int motorID)
{
    return configs[motorID];
}

void MotorDriver::setConfig(unsigned int motorID, SabertoothOperator config)
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
    //FUTURE:
    // read encoder data
    // send back encoder data
    // run PID loops
    read();
    parse();
    execute();
}
