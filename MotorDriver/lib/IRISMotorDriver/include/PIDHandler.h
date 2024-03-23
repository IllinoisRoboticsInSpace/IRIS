#ifndef PID_HANDLER
#define PID_HANDLER

#include <Arduino.h>

#undef min
#undef max
#include <array>


#include "PID.h"
#include "generated/commands.h"

#define DEFAULT_TUNINGS 0,0,0
#define DEFAULT_MOTOR_MAX 1
#define DEFAULT_MOTOR_MIN -1

class PIDHandler{ //rough outline 
    public:
        PIDHandler();

        float get_motor_value() const {return output_;}
        //void set_encoder_value(float encoder_value);
        void set_new_setpoint(float setpoint);

        bool update_pid(float input);
        //bool get_inverted() {return motor_pid.GetDirection();}
        void setEnabled(bool enabled);
        bool getEnabled() const {return enabled;}

        void set_sample_time(unsigned int sample_time);

        bool get_do_compute() const {return motor_pid.GetDoCompute();}

        bool get_in_control() const {return in_control;}
        byte get_motorID() const {return motor_ID_;}
        byte get_encoderID() const {return encoder_ID_;}
        //for some reason the get methods for PID are not const. they are literaly const, but they are not marked as such 
        float get_kd() {return motor_pid.GetKd();}
        float get_ki() {return motor_pid.GetKi();}
        float get_kp() {return motor_pid.GetKd();}
        float get_setPoint() const {return setpoint_;}

        bool applyConfigUpdate(const PID_Config_Data& update);
        bool applySetPoint(const Set_PID_Setpoint& update_setpoint);
        bool applyMotorControl(const Set_PID_Control& update_control);



    private:
        bool enabled;

        float input_ = 0;
        float output_ = 0; 
        float setpoint_ = 0;

        bool in_control;
        byte motor_ID_;
        byte encoder_ID_;
        PID motor_pid;
};


#endif