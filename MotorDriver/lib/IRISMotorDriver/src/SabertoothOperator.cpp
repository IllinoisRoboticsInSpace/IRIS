#include "SabertoothOperator.h"

SabertoothOperator::SabertoothOperator()
    : inverted(false), enabled(false), serialLine(&Serial1)
    , baudrate(DEFAULT_SABERTOOTH_BAUD_RATE), sabertooth(Sabertooth(130))
{
    
}

SabertoothOperator::SabertoothOperator(const SabertoothOperator& other)
    : inverted(other.inverted), enabled(other.enabled)
    , serialLine(other.serialLine), address(other.address), baudrate(other.baudrate)
    , sabertooth(other.sabertooth)
{
    //TODO: maybe make copy function to handle copying data in assignment operator and copy constructor
}

SabertoothOperator& SabertoothOperator::operator=(const SabertoothOperator& other)
{
    inverted = other.inverted;
    enabled = other.enabled;
    serialLine = other.serialLine;
    address = other.address;
    baudrate = other.baudrate;
    sabertooth = other.sabertooth;
    return *this;
}