#include "MotorDriver.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<MotorDriverConfig, NUM_ARDUINO_PINS> configs, Sabertooth *st)
{
    this->initialized = false;
    this->serialTransferBaudRate = serialTransferBaudRate;
    this->configs = configs;
    this->st = st;
}

MotorDriver::MotorDriver()
{
    MotorDriver(DEFAULT_SERIAL_TRANSFER_BAUD_RATE, std::array<MotorDriverConfig, NUM_ARDUINO_PINS>(), nullptr);
}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    if (st == nullptr)
    {
        return false;
    }
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

std::array<MotorDriver::MotorDriverConfig, NUM_ARDUINO_PINS> MotorDriver::getConfigs()
{
    return configs;
}

void MotorDriver::setConfigs(std::array<MotorDriverConfig, NUM_ARDUINO_PINS> configs)
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
    read();
    parse();
    execute();

    //FUTURE:
    //read encoder data
    //send back encoder data
    //run PID loops
}
