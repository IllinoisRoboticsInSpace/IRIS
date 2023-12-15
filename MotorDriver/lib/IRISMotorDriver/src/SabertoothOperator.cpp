#include "SabertoothOperator.h"
// #include "Sabertooth.h"
#include "DebugTools.h"

SabertoothOperator::SabertoothOperator(byte address, unsigned int baudrate, byte motornum, USARTClass& serial, bool inverted, bool enabled)
    : inverted(inverted), enabled(enabled), serialLine(serial), baudrate(baudrate), motornum(motornum), sabertooth(Sabertooth(address, serial))
{
    
}

SabertoothOperator::SabertoothOperator(byte address, unsigned int baudrate, byte motornum, USARTClass& serial)
    : inverted(false), enabled(false), serialLine(serial), baudrate(baudrate), motornum(motornum), sabertooth(Sabertooth(address, serial))
{

}

SabertoothOperator::SabertoothOperator()
    : inverted(false), enabled(false), serialLine(DEFAULT_SABERTOOTH_SERIAL_LINE)
    , baudrate(DEFAULT_SABERTOOTH_BAUD_RATE), motornum(DEFAULT_SABERTOOTH_MOTOR_NUM)
    , sabertooth(Sabertooth(DEFAULT_SABERTOOTH_ADDRESS, DEFAULT_SABERTOOTH_SERIAL_LINE))
{

}

SabertoothOperator::SabertoothOperator(const SabertoothOperator& other)
    : inverted(other.inverted), enabled(other.enabled)
    , serialLine(other.serialLine), baudrate(other.baudrate), motornum(other.motornum)
    , sabertooth(other.sabertooth)
{
    //TODO: maybe make copy function to handle copying data in assignment operator and copy constructor
}

SabertoothOperator& SabertoothOperator::operator=(const SabertoothOperator& other)
{
    inverted = other.inverted;
    enabled = other.enabled;
    serialLine = other.serialLine;
    baudrate = other.baudrate;
    motornum = other.motornum;
    sabertooth = other.sabertooth;
    return *this;
}

void SabertoothOperator::init()
{
    serialLine.begin(baudrate);
    sabertooth.autobaud();
}

void SabertoothOperator::setOutput(float percentOutput)
{
    if (inverted)
    {
        sabertooth.motor(motornum, -(byte) (percentOutput * SABERTOOTH_MAX_OUTPUT));
    }
    else
    {
        sabertooth.motor(motornum, (byte) (percentOutput * SABERTOOTH_MAX_OUTPUT));
    }
    //motor function will constraint output to -126 to 126.
    //power compute float times float and then casts to byte
    //casting to byte casts to integer to binary!
    //making SABERTOOTH_MAX_OUTPUT a byte so that we get float * byte
    //is not faster as it would require casting the byte to a float and then
    //casting back to a float once finished. This multiplies 2 floats and then casts to byte.
}

bool SabertoothOperator::applyConfigUpdate(const Sabertooth_Config_Data& update)
{
    auto key = update.get_which_values();
    switch (key)
    {
        case Sabertooth_Config_Data::FieldNumber::MOTORNUM:
            motornum = (byte) update.get_motorNum();
            break;
        case Sabertooth_Config_Data::FieldNumber::INVERTED:
            inverted = update.get_inverted();
            break;
        // TODO: Finish config updating consider when config updates should be applied
        // If address and serial line are changed then should init be run twice (once for each change) or once?
        // Maybe best solution is to require that operator is disbaled before applying config updates.
        default:
            return false;
            break;
    }
    return true;
}

void SabertoothOperator::setInverted(bool inverted)
{
    inverted = inverted;    //need to check if this is setting the input variable to itself or the instance field
}
void SabertoothOperator::setEnabled(bool enabled)
{
    enabled = enabled;
}