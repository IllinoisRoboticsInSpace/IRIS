#include "RotaryEncoderOperator.h"
#include "Arduino.h"

RotaryEncoderOperator(int pin1, int pin2, LatchMode mode = LatchMode::FOUR0){

    ENCODER_PIN_A = pin1;
    ENCODER_PIN_B = pin2;

    volatile int encoder0Pos = 0;
    volatile boolean PastB = 0;
    volatile boolean update = false;

    void RotaryEncoderOperator::interruptHandler(){
        pinMode(RotaryEncoderOperator::ENCODER_PIN_A, INPUT);
        pinMode(RotaryEncoderOperator::ENCODER_PIN_B, INPUT);
        attachInterrupt(0, encoderA, CHANGE);
        attachInterrupt(1, encoderB, CHANGE);
    }

    void RotaryEncoderOperator::encoderA(){
       RotaryEncoderOperator::encoderA()
        RotaryEncoderOperator::A_OLD = digitalRead(RotaryEncoderOperator::ENCODER_PIN_A);
    }

    void RotaryEncoderOperator::encoderB(){
        RotaryEncoderOperator::B_NEW = digitalRead(RotaryEncoderOperator::ENCODER_PIN_B);
        RotaryEncoderOperator::encoderA();
        B_NEW = digitalRead(encoder0PinB);
        Bnew^A_OLD ? encoder0Pos++ : encoder0Pos--;
    }
}