#ifndef PID_HANDLER
#define PID_HANDLER

#include <Arduino.h>

#undef min
#undef max
#include <array>


#include "PID_v1.h"
#include "generated/commands.h"

#define DEFAULT_TUNINGS 0,0,0
#define DEFAULT_MOTOR_MAX 1
#define DEFAULT_MOTOR_MIN 0

class PIDHandler{ //rough outline 
    public:
        PIDHandler(double kp, double ki, double kd, double motor_min, double motor_max, int inverted);
        PIDHandler();

        double get_motor_value() const {return output_;}
        //void set_encoder_value(double encoder_value);
        void set_new_setpoint(double setpoint);

        void update_pid(double input);
        

        void setInverted(bool inverted);
        void setEnabled(bool enabled);

        bool getControllingMotor() const {return controling_motor_;}
        byte getMotorID() const {return motor_ID_;}

        bool applyConfigUpdate(const PID_Config_Data& update);
        bool applySetPoint(const Set_PID_Setpoint& update_setpoint);
        bool applyMotorControl(const Set_PID_Control& update_control);



    private:
        bool inverted;
        bool enabled;

        double input_ = 0;
        double output_ = 0; 
        double setpoint_ = 0;

        bool controling_motor_;
        byte motor_ID_;
        PID motor_pid;
};


#endif