#include "PIDHandler.h"

PIDHandler::PIDHandler(double kp, double ki, double kd, double motor_min, double motor_max) : kp_(kp), ki_(ki), kd_(kd), motor_pid(&input_, &output_, &setpoint_, kp_, ki_, kd_, P_ON_E, DIRECT){
    motor_pid.SetOutputLimits(motor_min, motor_max);
    //motor_pid.SetSampleTime() set it to whatever samle time is best
}


double PIDHandler::get_motor_value() const {
    return output_;
}
void PIDHandler::set_encoder_value(double encoder_value) {
    input_ = encoder_value;
}
void PIDHandler::set_new_setpoint(double setpoint) {
    setpoint_ = setpoint;
}


void PIDHandler::set_new_kp(double kp) {kp_ = kp; set_tunings();}
void PIDHandler::set_new_ki(double ki) {ki_ = ki; set_tunings();}
void PIDHandler::set_new_kd(double kd) {kd_ = kd; set_tunings();}

void PIDHandler::update_pid() {
    motor_pid.Compute();
}

void PIDHandler::set_tunings(){
    motor_pid.SetTunings(kp_, ki_, kd_);
}