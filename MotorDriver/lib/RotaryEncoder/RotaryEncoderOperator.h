#include <iostream>
#include <list>
#include "RotaryEncoder.h"
#include <Arduino.h>

#ifndef _ENCODER_CONFIG_
    #define _ENCODER_CONFIG_
#endif

#define ENCODER_PIN_A
#define ENCODER_PIN_B
#define MOTOR_BUTTON_PIN
#define ENCODER_POS_0
#define TMP 
#define A_OLD
#define B_NEW

#define list<RotaryEncoder> encoderList;
#define TICKS_TO_DEG 0.3515625 
#define ENCODER_READ 0
#define PREV_ENCODER_READ 0
#define ENCODER_READ_LONG 0

#define LATCH_MODE




class RotaryEncoderOperator {
    public:

    RotaryEncoderOperator(int pin1, int pin2, LatchMode mode = LatchMode::FOUR0, bool inverted, bool enabled);
    RotaryEncoderOperator();

    bool init();
    void interruptHandler();
    void encoderA();
    void encoderB();
    bool applyConfigUpdate(const Encoder_Config_Data& update);
    byte getEncoderID() const {return encoder_ID;}
    
    private:
    byte encoder_ID;

}
