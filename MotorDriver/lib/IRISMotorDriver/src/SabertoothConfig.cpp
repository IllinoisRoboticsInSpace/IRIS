#include "SabertoothConfig.h"

SabertoothConfig::SabertoothConfig()
    : inverted(false), enabled(false), serialLine(&Serial1)
    , baudrate(DEFAULT_SABERTOOTH_BAUD_RATE), sabertooth(Sabertooth(130))
{
    
}

SabertoothConfig::SabertoothConfig(const SabertoothConfig& other)
    : motorNum(other.motorNum), inverted(other.inverted), enabled(other.enabled)
    , serialLine(other.serialLine), address(other.address), baudrate(other.baudrate)
    , sabertooth(other.sabertooth)
{
    //TODO: maybe make copy function to handle copying data in assignment operator and copy constructor
}

SabertoothConfig& SabertoothConfig::operator=(const SabertoothConfig& other)
{
    motorNum = other.motorNum;
    inverted = other.inverted;
    enabled = other.enabled;
    serialLine = other.serialLine;
    address = other.address;
    baudrate = other.baudrate;
    sabertooth = other.sabertooth;
    return *this;
}