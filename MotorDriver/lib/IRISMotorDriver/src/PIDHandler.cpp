#include "PIDHandler.h"
#include "DebugTools.h"
#include "ProtobufUtilities.h"

PIDHandler::PIDHandler() : motor_pid(&input_, &output_, &setpoint_, DEFAULT_TUNINGS, true, false), enabled(false), in_control(false) {
    motor_pid.SetOutputLimits(DEFAULT_MOTOR_MIN, DEFAULT_MOTOR_MAX);
}


void PIDHandler::set_new_setpoint(double setpoint) {
    setpoint_ = setpoint;
}

void PIDHandler::setEnabled(bool enabled){
    this->enabled = enabled;
}

bool PIDHandler::update_pid(double input) {
    this->input_ = input;
    return motor_pid.Compute();
}

bool PIDHandler::applyConfigUpdate(const PID_Config_Data& update)
{   
    DEBUG_PRINTLN("Applying PID Config Update:")
    DEBUG_PRINT_MESSAGE(update)
    DEBUG_PRINTLN("")
    
    // Always apply enabled change
    auto key = update.get_which_values();
    if (key == PID_Config_Data::FieldNumber::ENABLED){
        setEnabled(update.get_enabled());
        return true;
    }

    // Not allowed to apply config update to enabled operator
    if (enabled == false)
    {
        return false;
    }

    switch (key)
    {
        case PID_Config_Data::FieldNumber::MOTORID:
            {
                motor_ID_ = update.get_motorID();
                break;
            }
        case PID_Config_Data::FieldNumber::KP:
            {
                motor_pid.SetTunings(update.get_kp(), motor_pid.GetKi(), motor_pid.GetKd());
                break;
            }
        case PID_Config_Data::FieldNumber::KI:
            {
                motor_pid.SetTunings(motor_pid.GetKp(), update.get_ki(), motor_pid.GetKd());
                break;
            }
        case PID_Config_Data::FieldNumber::KD:
            {
                motor_pid.SetTunings(motor_pid.GetKp(), motor_pid.GetKi(), update.get_kd());
                break;
            }
        default:
            return false;
            break;
    }
    return true;
}
bool PIDHandler::applySetPoint(const Set_PID_Setpoint& update_setpoint){
    setpoint_ = update_setpoint.get_setPoint();
}
bool PIDHandler::applyMotorControl(const Set_PID_Control& update_control){
    in_control = update_control.get_in_control();
}