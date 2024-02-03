#include "SabertoothOperator.h"
#include "DebugTools.h"
#include "ProtobufUtilities.h"
#include <mutex>

SabertoothOperator::SabertoothOperator(byte address, unsigned int baudrate, byte motornum, USARTClass& serial, bool inverted, bool enabled)
    : MotorOperator(inverted, enabled), serialLine(serial), baudrate(baudrate), motornum(motornum), sabertooth(Sabertooth(address, serial))
{
    
}

SabertoothOperator::SabertoothOperator(byte address, unsigned int baudrate, byte motornum, USARTClass& serial)
    : MotorOperator(false, false), serialLine(serial), baudrate(baudrate), motornum(motornum), sabertooth(Sabertooth(address, serial))
{

}

SabertoothOperator::SabertoothOperator()
    : MotorOperator(false, false), serialLine(DEFAULT_SABERTOOTH_SERIAL_LINE)
    , baudrate(DEFAULT_SABERTOOTH_BAUD_RATE), motornum(DEFAULT_SABERTOOTH_MOTOR_NUM)
    , sabertooth(Sabertooth(DEFAULT_SABERTOOTH_ADDRESS, DEFAULT_SABERTOOTH_SERIAL_LINE))
{

}

SabertoothOperator::SabertoothOperator(const SabertoothOperator& other)
    : MotorOperator(other.inverted, other.enabled)
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

bool SabertoothOperator::init()
{
    if (enabled == true)
    {
        serialLine.begin(baudrate);
        sabertooth.autobaud();
    }
    return enabled;
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

/**
 * Will update the state of the sabertooth operator based on the provided Sabertooth_Config_Data.
 * Will return false if the sabertooth operator is not enabled or if the Sabertooth_Config_Data
 * does not contain data. 
 * @param update protobuf message with config update data
*/
bool SabertoothOperator::applyConfigUpdate(const Sabertooth_Config_Data& update)
{   
    DEBUG_PRINTLN("Applying Config Update:")
    DEBUG_PRINT_MESSAGE(update)
    DEBUG_PRINTLN("")
    
    // Always apply enabled change
    auto key = update.get_which_values();
    if (key == Sabertooth_Config_Data::FieldNumber::ENABLED)
    {
        setEnabled(update.get_enabled());
        if (enabled == true)
        {
            init();
        }
        return true;
    }

    // Not allowed to apply config update to enabled operator
    if (enabled == false)
    {
        return false;
    }

    switch (key)
    {
        case Sabertooth_Config_Data::FieldNumber::INVERTED:
            inverted = update.get_inverted();
            break;
        case Sabertooth_Config_Data::FieldNumber::MOTORNUM:
            motornum = (byte) update.get_motorNum();
            break;
        case Sabertooth_Config_Data::FieldNumber::ADDRESS:
            {
            sabertooth = Sabertooth(update.get_address(), serialLine);
            break;
            }
        case Sabertooth_Config_Data::FieldNumber::SERIALLINE:
            {
            auto newSerialLine = getSabertoothSerial(update.get_serialLine());
            sabertooth = Sabertooth(update.get_address(), newSerialLine);
            break;
            }
        default:
            return false;
            break;
    }
    return true;
}
