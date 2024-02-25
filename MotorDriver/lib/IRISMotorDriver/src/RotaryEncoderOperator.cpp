#include "RotaryEncoderOperator.h"
#include "DebugTools.h"

RotaryEncoderOperator::RotaryEncoderOperator(int pin1, int pin2, RotaryEncoder::LatchMode mode)
    : enabled(false), pin_In(pin1), pin_Out(pin2), _old_pin_In(-1), _old_pin_Out(-1)
    , latch_Mode(mode)
{
    encoder = new RotaryEncoder(pin_In, pin_Out, latch_Mode);
}

RotaryEncoderOperator::RotaryEncoderOperator()
    : enabled(false), pin_In(DEFAULT_PIN1), pin_Out(DEFAULT_PIN2), _old_pin_In(-1), _old_pin_Out(-1)
    , latch_Mode(DEFAULT_LATCHMODE)
{
    encoder = new RotaryEncoder(DEFAULT_PIN1, DEFAULT_PIN2, DEFAULT_LATCHMODE);
}

void RotaryEncoderOperator::pin1InterruptHandler()
{
    encoder->tick();
}

void RotaryEncoderOperator::pin2InterruptHandler()
{
    encoder->tick();
}

bool RotaryEncoderOperator::init()
{
    if (enabled == true)
    {
        reallocateInterruptHandlers();
        
    }
    return enabled;
}

// Only to be called during init
// A simple reallocation procedure is done.
bool RotaryEncoderOperator::reallocateInterruptHandlers()
{
    if (pin_In != _old_pin_In) // Update to pin 1 has been requested
    {
        if ((digitalPinAllocations[pin_In] == false) && (pin_In != -1)) // Pin is available
        {
            // Detach old interrupt
            if (_old_pin_In != -1)
            {
                detachInterrupt(digitalPinToInterrupt(_old_pin_In));
            }
            // Free object binding
            if (interruptGate_pin1 != nullptr)
            {
                bindArgGateFree(interruptGate_pin1);
            }
            // Deallocate resource
            digitalPinAllocations[_old_pin_In] = false;
                  
            // Get the member function pointer for interrupt handler with object instance.
            interruptGate_pin1 = bindArgGateThisAllocate(&RotaryEncoderOperator::pin1InterruptHandler, this);

            // Setup pin interrupt
            attachInterrupt(digitalPinToInterrupt(pin_In), interruptGate_pin1, CHANGE);

            // Allocate resource
            digitalPinAllocations[pin_In] = true;
        }
    }

    if (pin_Out != _old_pin_Out) // Update to pin2 has been requested
    {
        if ((digitalPinAllocations[pin_Out] == false) && (pin_Out != -1)) // Pout is available
        {
            // Detach old interrupt
            if (_old_pin_Out != -1)
            {
                detachInterrupt(digitalPinToInterrupt(_old_pin_Out));
            }
            // Free object binding
            if (interruptGate_pin2 != nullptr)
            {
                bindArgGateFree(interruptGate_pin2);
            }
            // Deallocate resource
            digitalPinAllocations[_old_pin_Out] = false;
                  
            // Get the member function pointer for interrupt handler with object instance.
            interruptGate_pin2 = bindArgGateThisAllocate(&RotaryEncoderOperator::pin2InterruptHandler, this);

            // Setup pin interrupt
            attachInterrupt(digitalPinToInterrupt(pin_Out), interruptGate_pin2, CHANGE);

            // Allocate resource
            digitalPinAllocations[pin_Out] = true;
        }
    }
}


bool RotaryEncoderOperator::update_Encoder(double input){
    // ENCODER_READ_LONG = encoder.getPosition();
    // ENCODER_READ = (double)ENCODER_READ_LONG;
}

bool RotaryEncoderOperator::applyConfigUpdate(const Encoder_Config_Data& update){
    DEBUG_PRINTLN("Applying Rotary Encoder Configs")
    DEBUG_PRINT_MESSAGE(update)
    DEBUG_PRINTLN("")

    auto key = update.get_which_values();
    if(key == Encoder_Config_Data::FieldNumber::ENABLED){
        //setEnabled(update.get_enabled()); //TODO: Implement in RotaryEncoderOperator
        return true;
    }
    if(enabled==false){
        return false;
    }

    switch(key){
        case Encoder_Config_Data::FieldNumber::LATCHMODE:{
            // latch_Mode = update.get_latchMode();
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
