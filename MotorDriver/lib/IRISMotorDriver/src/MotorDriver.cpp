#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothConfig, MAX_MOTOR_CONFIGS> configs)
    : initialized(false)
{
    this->serialTransferBaudRate = serialTransferBaudRate;
    this->configs = configs;
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

std::array<SabertoothConfig, MAX_MOTOR_CONFIGS> MotorDriver::getConfigs()
{
    return configs;
}

void MotorDriver::setConfigs(std::array<SabertoothConfig, MAX_MOTOR_CONFIGS> configs)
{
    this->configs = configs;
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
    //read encoder data
    //send back encoder data
    //run PID loops
    read();
    parse();
    execute();
}
