#include "RotaryEncoderOperator.h"
#include "DebugTools.h"

std::array<bool, NUM_DIGITAL_PINS> RotaryEncoderOperator::digitalPinAllocations;

RotaryEncoderOperator::RotaryEncoderOperator(int pin1, int pin2, RotaryEncoder::LatchMode mode)
    : enabled(false), new_pin_In(pin1), new_pin_Out(pin2), _current_pin_In(-1), _current_pin_Out(-1)
    , latch_Mode(mode)
{
    encoder = new RotaryEncoder(new_pin_In, new_pin_Out, latch_Mode);
}

RotaryEncoderOperator::RotaryEncoderOperator()
    : enabled(false), new_pin_In(DEFAULT_PIN1), new_pin_Out(DEFAULT_PIN2), _current_pin_In(-1), _current_pin_Out(-1)
    , latch_Mode(DEFAULT_LATCHMODE)
{
    encoder = new RotaryEncoder(DEFAULT_PIN1, DEFAULT_PIN2, DEFAULT_LATCHMODE);
}

void RotaryEncoderOperator::setInverted(bool inverted)
{
    this->inverted = inverted;
}

void RotaryEncoderOperator::setEnabled(bool enabled)
{
    this->enabled = enabled;
}

bool RotaryEncoderOperator::getInverted()
{
    return inverted;
}

bool RotaryEncoderOperator::getEnabled()
{
    return enabled;
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
        bool successfull = reallocateInterruptHandlers();

    }
    return enabled;
}

// Only to be called during init
// A simple reallocation procedure is done.
bool RotaryEncoderOperator::reallocateInterruptHandlers()
{
    if (new_pin_In != _current_pin_In && (new_pin_In != -1)) // Update to pin 1 has been requested
    {
        if ((new_pin_In != -1) && (RotaryEncoderOperator::digitalPinAllocations[new_pin_In] == false)) // Pin is available
        {
            // Detach old interrupt
            if (_current_pin_In != -1)
            {
                detachInterrupt(digitalPinToInterrupt(_current_pin_In));
            }
            // Free object binding
            if (interruptGate_pin1 != nullptr)
            {
                bindArgGateFree(interruptGate_pin1);
            }
            // Deallocate resource
            RotaryEncoderOperator::digitalPinAllocations[_current_pin_In] = false;
                  
            // Get the member function pointer for interrupt handler with object instance.
            interruptGate_pin1 = bindArgGateThisAllocate(&RotaryEncoderOperator::pin1InterruptHandler, this);

            // Setup pin interrupt
            attachInterrupt(digitalPinToInterrupt(new_pin_In), interruptGate_pin1, CHANGE);

            // Allocate resource
            RotaryEncoderOperator::digitalPinAllocations[new_pin_In] = true;
            _current_pin_In = new_pin_In;
        }
    }

    if (new_pin_Out != _current_pin_Out && (new_pin_Out != -1)) // Update to pin2 has been requested
    {
        if ((new_pin_Out != -1) && (RotaryEncoderOperator::digitalPinAllocations[new_pin_Out] == false)) // Pout is available
        {
            // Detach old interrupt
            if (_current_pin_Out != -1)
            {
                detachInterrupt(digitalPinToInterrupt(_current_pin_Out));
            }
            // Free object binding
            if (interruptGate_pin2 != nullptr)
            {
                bindArgGateFree(interruptGate_pin2);
            }
            // Deallocate resource
            RotaryEncoderOperator::digitalPinAllocations[_current_pin_Out] = false;
                  
            // Get the member function pointer for interrupt handler with object instance.
            interruptGate_pin2 = bindArgGateThisAllocate(&RotaryEncoderOperator::pin2InterruptHandler, this);

            // Setup pin interrupt
            attachInterrupt(digitalPinToInterrupt(new_pin_Out), interruptGate_pin2, CHANGE);

            // Allocate resource
            RotaryEncoderOperator::digitalPinAllocations[new_pin_Out] = true;
            _current_pin_Out = new_pin_Out;
        }
    }

    return true;//TODO: Add actual failure logic
}

bool RotaryEncoderOperator::deallocateInterruptHandlers()
{
    // Set default allocation pin
    // On enable, the previous pins will be restored and their interrupts set
    new_pin_In = _current_pin_In;
    new_pin_Out = _current_pin_Out;

    if (_current_pin_In != -1 && RotaryEncoderOperator::digitalPinAllocations[_current_pin_In] == true)
    {
        // Detach old interrupt
        detachInterrupt(digitalPinToInterrupt(_current_pin_In));
        // Free object binding
        bindArgGateFree(interruptGate_pin1);
        interruptGate_pin1 = nullptr;
        // Deallocate resource
        RotaryEncoderOperator::digitalPinAllocations[_current_pin_In] = false;
    }

    if (_current_pin_Out != -1 && RotaryEncoderOperator::digitalPinAllocations[_current_pin_Out] == true)
    {
        // Detach old interrupt
        detachInterrupt(digitalPinToInterrupt(_current_pin_Out));
        // Free object binding
        bindArgGateFree(interruptGate_pin2);
        interruptGate_pin2 = nullptr;
        // Deallocate resource
        RotaryEncoderOperator::digitalPinAllocations[_current_pin_Out] = false;
    }

    _current_pin_In = -1;
    _current_pin_Out = -1;

    return true;//TODO: Add failure logic if deemed necessary
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
    if (key == Encoder_Config_Data::FieldNumber::ENABLED)
    {
        setEnabled(update.get_enabled());
        if (enabled == true)
        {
            init();
        }
        else
        {
            bool successful_disable = deallocateInterruptHandlers();
        }
        return true;
    }
    
    // Not allowed to apply config update to enabled operator
    if (enabled == true)
    {
        return false;
    }

    switch(key){
        case Encoder_Config_Data::FieldNumber::LATCHMODE:{
            // latch_Mode = update.get_latchMode();
            break;
        }
        case Encoder_Config_Data::FieldNumber::PININ:{
            new_pin_In = update.get_pinIn();
            break;
        }
        case Encoder_Config_Data::FieldNumber::PINOUT:{
            new_pin_Out = update.get_pinOut();
            break;
        }
       
    }
}
