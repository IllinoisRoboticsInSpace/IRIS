#include "PIDHandler.h"
#include "DebugTools.h"
#include "ProtobufUtilities.h"

PIDHandler::PIDHandler(double kp, double ki, double kd, double motor_min, double motor_max, int inverted) : inverted(false), enabled(false),
    motor_pid(&input_, &output_, &setpoint_, kp, ki, kd, P_ON_E, inverted)
{
    motor_pid.SetOutputLimits(motor_min, motor_max);
    //motor_pid.SetSampleTime() set it to whatever samle time is best
}
PIDHandler::PIDHandler() : motor_pid(&input_, &output_, &setpoint_, DEFAULT_TUNINGS, P_ON_E, inverted){

}

// double PIDHandler::get_motor_value() const {
//     return output_;
// }
// void PIDHandler::set_encoder_value(double encoder_value) {
//     input_ = encoder_value;
// }
void PIDHandler::set_new_setpoint(double setpoint) {
    setpoint_ = setpoint;
}

// void PIDHandler::set_new_kp(double kp) {kp_ = kp; set_tunings();}
// void PIDHandler::set_new_ki(double ki) {ki_ = ki; set_tunings();}
// void PIDHandler::set_new_kd(double kd) {kd_ = kd; set_tunings();}

void PIDHandler::setInverted(bool inverted){
    this->inverted = inverted;
}
void PIDHandler::setEnabled(bool enabled){
    this->enabled = enabled;
}
// bool PIDHandler::getEnabled(){
//     return this->enabled;
// }


void PIDHandler::update_pid(double input) {
    this->input_ = input;
    motor_pid.Compute();
}

// void PIDHandler::set_tunings(){
//     motor_pid.SetTunings(kp_, ki_, kd_);
// }

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
        case PID_Config_Data::FieldNumber::INVERTED:{
                motor_pid.SetControllerDirection(update.get_inverted());
                break;
            }
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
    setpoint_ = update_setpoint.get_set_point();
}
bool PIDHandler::applyMotorControl(const Set_PID_Control& update_control){
    controling_motor_ = update_control.get_pid_control_enabled();
}