#ifndef PID_HANDLER
#define PID_HANDLER

#include "PID_v1.h"

class PIDHandler{ //rough outline 
    public:
        PIDHandler(double kp, double ki, double kd, double motor_min, double motor_max);


        double get_motor_value() const;
        void set_encoder_value(double encoder_value);
        void set_new_setpoint(double setpoint);

        void update_pid();
        
        void set_new_kp(double kp) {motor_pid.SetTunings(kp, motor_pid.GetKi(), motor_pid.GetKd());};
        void set_new_ki(double ki) {motor_pid.SetTunings(motor_pid.GetKp(), ki, motor_pid.GetKd());};
        void set_new_kd(double kd) {motor_pid.SetTunings(motor_pid.GetKp(), motor_pid.GetKi(), kd);};

    private:
        double input_ = 0;
        double output_ = 0; 
        double setpoint_ = 0;

        double kp_;
        double ki_; 
        double kd_;

        void set_tunings();

        PID motor_pid;
};


#endif