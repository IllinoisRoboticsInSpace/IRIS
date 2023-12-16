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
    : serialTransferBaudRate(serialTransferBaudRate), configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
{
    
}

MotorDriver::MotorDriver()
    : serialTransferBaudRate(DEFAULT_HOST_SERIAL_BAUD_RATE), configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
{

}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    // Check if enabled configs need initialization.
    for (SabertoothOperator sabertoothOperator : configs)
    {
        // Configs can be set and enabled before serial connection is made
        // Otherwise configs can only become enabled by host driver.
        if (sabertoothOperator.getEnabled() == true)
        {
            sabertoothOperator.init();
        }
    }
    Serial.begin(serialTransferBaudRate); //Serial used for USB is reserved for communication with host
    return true;
}

SabertoothOperator MotorDriver::getConfig(unsigned int motorID)
{
    return configs[motorID];
}

void MotorDriver::setConfig(unsigned int motorID, SabertoothOperator config)
{
    configs[motorID] = config;  // Does not init with new config init motor driver must be called again.
}

void MotorDriver::resetConfigs()
{
    // Maybe in future maintain the type of the config
    configs = std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>();
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
