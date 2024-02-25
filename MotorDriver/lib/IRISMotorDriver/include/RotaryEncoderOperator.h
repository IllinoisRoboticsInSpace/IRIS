#ifndef _ENCODER_CONFIG_
#define _ENCODER_CONFIG_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "RotaryEncoder.h"
#include "ProtobufUtilities.h"
#include "generated/commands.h"


#define TICKS_TO_DEG 0.3515625 
#define ENCODER_READ 0
#define PREV_ENCODER_READ 0
#define ENCODER_READ_LONG 0

#define DEFAULT_PIN1 22
#define DEFAULT_PIN2 23
#define DEFAULT_LATCHMODE RotaryEncoder::LatchMode::FOUR3

class RotaryEncoderOperator {
    public:
        RotaryEncoderOperator(int pin1, int pin2, RotaryEncoder::LatchMode mode);
        RotaryEncoderOperator();
        bool update_Encoder(double input);

        bool init();
        bool applyConfigUpdate(const Encoder_Config_Data& update);
    
    private:
        //General to encoders
        bool enabled;
        // int8_t motor_ID; //Does not necessarily exist because encoder could be on axle that does not have motor. So default is -1.

        // Rotary Encoder specific
        int pin_In;
        int pin_Out;
        RotaryEncoder::LatchMode latch_Mode;
};
#endif