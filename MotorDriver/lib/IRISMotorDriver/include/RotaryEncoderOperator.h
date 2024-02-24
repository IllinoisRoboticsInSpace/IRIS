#ifndef _ENCODER_CONFIG_
#define _ENCODER_CONFIG_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "RotaryEncoder.h"
#include "generated/commands.h"



#define TICKS_TO_DEG 0.3515625 
#define ENCODER_READ 0
#define PREV_ENCODER_READ 0
#define ENCODER_READ_LONG 0



class RotaryEncoderOperator {
    public:

    RotaryEncoderOperator(int pin1, int pin2, RotaryEncoder::LatchMode mode);
    RotaryEncoderOperator();
    bool update_Encoder(double input);

    bool init();
    bool applyConfigUpdate(const Encoder_Config_Data& update);
    byte getEncoderID() {return encoder_ID;}
    
    private:
    bool enabled;

    byte motor_ID;
    byte encoder_ID;
    int pin_In;
    int pin_Out;
    RotaryEncoder::LatchMode latch_Mode;

};
#endif