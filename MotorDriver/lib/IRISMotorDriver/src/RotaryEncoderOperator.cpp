#include "RotaryEncoderOperator.h"
#include "DebugTools.h"
#include "ProtobufUtilities.h"

RotaryEncoderOperator::RotaryEncoderOperator(int pin1, int pin2, LatchMode mode = LatchMode::FOUR0);
bool RotaryEncoderOperator::update_Encoder(double input){
    ENCODER_READ_LONG = encoder.getPosition();
    ENCODER_READ = (double)ENCODER_READ_LONG;
}

bool RotaryEncoderOperator::applyConfigUpdate(const Encoder_Config_Data& update){
    DEBUG_PRINTLN("Applying Rotary Encoder Configs")
    DEBUG_PRINT_MESSAGE(update)
    DEBUG_PRINTLN("")

    auto key = update.get_which_values();
    if(key == Encoder_Config_Data::FieldNumber::ENABLED){
        setEnabled(update.get_enabled());
        return true;
    }
    if(enabled==false){
        return false;
    }

    switch(key){
        case Encoder_Config_Data::FieldNumber::MOTORID:{
            motor_ID = update.get_motorID();
            break;
        }
        case Encoder_Config_Data::FieldNumber::LATCHMODE:{
            latch_Mode = update.get_latchMode();
            break;
        }
        case Encoder_Config_Data::FieldNumber::PININ:{
            pin_In = update.get_pinIn();
            break;
        }
        case Encoder_Config_Data::FieldNumber::PINOUT:{
            pin_Out = update.get_pinOut();
            break;
        }
       
    }
}