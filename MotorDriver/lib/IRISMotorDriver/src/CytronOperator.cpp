#include "CytronOperator.h"
#include "DebugTools.h"
#include "ProtobufUtilities.h"
#include <mutex>

CytronOperator::CytronOperator()
    : MotorOperator(false, false), pwm_pin(DEFAULT_CYTRON_MOTOR_PWM_PIN), dir_pin(DEFAULT_CYTRON_MOTOR_DIR_PIN){
        init();
    }

CytronOperator::CytronOperator(byte pin_1, byte pin_2, bool _inverted, bool _enabled) 
    :  MotorOperator(_inverted, _enabled), pwm_pin(pin_1), dir_pin(pin_2){
    init();
}
CytronOperator::CytronOperator(byte pin_1, byte pin_2)
    : MotorOperator(false, false), pwm_pin(pin_1), dir_pin(pin_2){
    init();
}
CytronOperator::CytronOperator(const CytronOperator& other)
    : MotorOperator(other.inverted, other.enabled), pwm_pin(other.pwm_pin), dir_pin(other.dir_pin) {
    init();
}

CytronOperator& CytronOperator::operator=(const CytronOperator& other)
{
    if(this == &other){
        return *this;
    }
    inverted = other.inverted;
    enabled = other.enabled;
    pwm_pin = other.pwm_pin;
    dir_pin = other.dir_pin;
    return *this;
}

bool CytronOperator::init()
{
    if (enabled == true){
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        digitalWrite(pwm_pin, LOW);
        digitalWrite(dir_pin, LOW);
    }
    return enabled;
}

void CytronOperator::setOutput(float percentOutput){
    if(inverted != percentOutput <= 0){ // motor moves backwards if the commands are inverted or the percent output is negative
        digitalWrite(dir_pin, HIGH);
    }
    else{
        digitalWrite(dir_pin, LOW);
    }
    if(percentOutput > 1){
        percentOutput = 1;
    }
    if(percentOutput < -1){
        percentOutput = -1;
    }
    if(percentOutput < 0){
        percentOutput *= -1;
    }
    analogWrite(pwm_pin, (byte)(percentOutput * CYTRON_MAX_OUTPUT));
}

/**
 * Will update the state of the cytron operator based on the provided Cytron_Config_Data.
 * Will return false if the cytron operator is not enabled or if the Cytron_Config_Data
 * does not contain data. 
 * @param update protobuf message with config update data
*/
bool CytronOperator::applyConfigUpdate(const Cytron_Config_Data& update)
{   
    DEBUG_PRINTLN("Applying Config Update:")
    DEBUG_PRINT_MESSAGE(update)
    DEBUG_PRINTLN("")
    
    // Always apply enabled change
    auto key = update.get_which_values();
    if (key == Cytron_Config_Data::FieldNumber::ENABLED)
    {
        setEnabled(update.get_enabled());
        if (enabled == true)
        {
            init();
        }
        return true;
    }

    // Not allowed to apply config update to enabled operator
    if (enabled == true)
    {
        return false;
    }

    switch (key)
    {
        case Cytron_Config_Data::FieldNumber::INVERTED:
        {
            inverted = update.get_inverted();
            break;
        }
        case Cytron_Config_Data::FieldNumber::PWM_PIN:
        {
            pwm_pin = (byte) update.get_pwm_pin();
            break;
        }
        case Cytron_Config_Data::FieldNumber::DIR_PIN:
        {
            dir_pin = (byte) update.get_dir_pin();
            break;
        }  
        default:
        {
            return false;
            break;
        }
    }
    return true;
}
byte CytronOperator::get_pwm_pin() const {return pwm_pin;}
byte CytronOperator::get_dir_pin() const {return dir_pin;}
