#ifndef _ENCODER_CONFIG_
#define _ENCODER_CONFIG_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "RotaryEncoder.h"
#include "ProtobufUtilities.h"
#include "generated/commands.h"
#include "BindArg.h"


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

        // general
        void setInverted(bool inverted);
        void setEnabled(bool enabled);
        bool getInverted();
        bool getEnabled();

        bool init();

        // Specific
        bool applyConfigUpdate(const Encoder_Config_Data& update);

        void pin1InterruptHandler();
        void pin2InterruptHandler();
    
    private:
        //General to encoders
        bool enabled;
        bool inverted;

        // Rotary Encoder specific
        int new_pin_In, new_pin_Out;
        int _current_pin_In, _current_pin_Out;  // Used for detaching previous interrupt
        RotaryEncoder* encoder;

        // Manage digital pin interrupt allocation
        bool reallocateInterruptHandlers();
        bool deallocateInterruptHandlers();
        // https://stackoverflow.com/questions/56389249/how-to-use-a-c-member-function-as-an-interrupt-handler-in-arduino
        // https://github.com/openlab-vn-ua/BindArg
        bindArgVoidFunc_t interruptGate_pin1 = nullptr;
        bindArgVoidFunc_t interruptGate_pin2 = nullptr;

        // IMPORTANT NOTE:
        // Need to be careful with resource allocations order because
        // Reallocating multiple resources requires order checking
        // to avoid conflicts of allocating to pins that are scheduled
        // to be deallocated
        static std::array<bool, NUM_DIGITAL_PINS> digitalPinAllocations;
};
#endif