#include "CytronOperator.h"
#include "DebugTools.h"
#include "ProtobufUtilities.h"
#include <mutex>

CytronOperator::CytronOperator()
    : MotorOperator(false, false), motornum(DEFAULT_SABERTOOTH_MOTOR_NUM)
{

}

CytronOperator::CytronOperator(const CytronOperator& other)
    : MotorOperator(other.inverted, other.enabled), motornum(other.motornum)
{
    //TODO: maybe make copy function to handle copying data in assignment operator and copy constructor
}

CytronOperator& CytronOperator::operator=(const CytronOperator& other)
{
    inverted = other.inverted;
    enabled = other.enabled;
    motornum = other.motornum;
    return *this;
}

bool CytronOperator::init()
{
    if (enabled == true)
    {
        // Initialization
    }
    return enabled;
}

void CytronOperator::setOutput(float percentOutput)
{
    if (inverted)
    {
        
    }
    else
    {
        
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
// bool CytronOperator::applyConfigUpdate(const Sabertooth_Config_Data& update)
// {   
//     DEBUG_PRINTLN("Applying Config Update:")
//     DEBUG_PRINT_MESSAGE(update)
//     DEBUG_PRINTLN("")
    
//     // Always apply enabled change
//     auto key = update.get_which_values();
//     if (key == Sabertooth_Config_Data::FieldNumber::ENABLED)
//     {
//         setEnabled(update.get_enabled());
//         if (enabled == true)
//         {
//             init();
//         }
//         return true;
//     }

//     // Not allowed to apply config update to enabled operator
//     if (enabled == true)
//     {
//         return false;
//     }

//     switch (key)
//     {
//         case Sabertooth_Config_Data::FieldNumber::INVERTED:
//             inverted = update.get_inverted();
//             break;
//         case Sabertooth_Config_Data::FieldNumber::MOTORNUM:
//             motornum = (byte) update.get_motorNum();
//             break;
//         default:
//             return false;
//             break;
//     }
//     return true;
// }
