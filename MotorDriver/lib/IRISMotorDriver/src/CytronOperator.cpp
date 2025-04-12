// #include "CytronOperator.h"
// #include "DebugTools.h"
// #include "ProtobufUtilities.h"


// CytronOperator::CytronOperator(uint8_t pwmPin, uint8_t dirPin, bool inverted, bool enabled)
//     : MotorOperator(inverted, enabled), pwmPin(pwmPin), cytronDriver(nullptr)
// {
// }

// CytronOperator::CytronOperator(uint8_t pwmPin, uint8_t dirPin)
//     : MotorOperator(false, false), pwmPin(pwmPin), dirPin(dirPin), cytronDriver(nullptr)
// {
// }

// CytronOperator::CytronOperator()
//     : MotorOperator(false, false), pwmPin(0), dirPin(0), cytronDriver(nullptr)
// {
// }

// CytronOperator::CytronOperator(const CytronOperator& other)
//     : MotorOperator(other), pwmPin(other.pwmPin), dirPin(other.dirPin), cytronDriver(nullptr)
// {
// }

// CytronOperator& CytronOperator::operator=(const CytronOperator& other)
// {
//     if (this != &other)
//     {
//         MotorOperator::operator=(other);
//         pwmPin = other.pwmPin;
//         dirPin = other.dirPin;
        
//         // Clean up existing driver if any
//         if (cytronDriver != nullptr)
//         {
//             delete cytronDriver;
//             cytronDriver = nullptr;
//         }
//     }
//     return *this;
// }

// bool CytronOperator::init()
// {
//     if (!enabled)
//     {
//         DEBUG_PRINTLN("Cytron motor not enabled");
//         return false;
//     }

//     // Clean up existing driver if any
//     if (cytronDriver != nullptr)
//     {
//         delete cytronDriver;
//         cytronDriver = nullptr;
//     }

//     cytronDriver = new CytronMD(PWM_DIR, pwmPin, dirPin);

//     // Initialize with zero speed
//     cytronDriver->setSpeed(0);
    
//     DEBUG_PRINTLN("Cytron motor initialized");
//     return true;
// }

// void CytronOperator::setOutput(float percentOutput)
// {
//     if (!enabled || cytronDriver == nullptr)
//     {
//         return;
//     }

//     // Clamp percentOutput between -1.0 and 1.0
//     if (percentOutput > 1.0f)
//     {
//         percentOutput = 1.0f;
//     }
//     else if (percentOutput < -1.0f)
//     {
//         percentOutput = -1.0f;
//     }

//     // Apply inversion if needed
//     if (inverted)
//     {
//         percentOutput = -percentOutput;
//     }

//     // Convert from -1.0 to 1.0 range to -255 to 255 range
//     int16_t speed = static_cast<int16_t>(percentOutput * 255.0f);
    
//     // Set the speed on the Cytron driver
//     cytronDriver->setSpeed(speed);
// }

// bool CytronOperator::applyConfigUpdate(const Cytron_Config_Data& update)
// {
//     // Handle different configuration updates
//     if (update.has_inverted())
//     {
//         inverted = update.inverted();
//     }
//     else if (update.has_enabled())
//     {
//         enabled = update.enabled();
//         if (enabled)
//         {
//             return init();
//         }
//     }
//     else if (update.has_pwmPin())
//     {
//         pwmPin = update.pwmPin();
//         return init();
//     }
//     else if (update.has_dirPin())
//     {
//         dirPin = update.dirPin();
//         return init();
//     }
//     else if (update.has_pwm2Pin())
//     {
//         dirPin = update.pwm2Pin();
//         return init();
//     }
    
//     return true;
// }
