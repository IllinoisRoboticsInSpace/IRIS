#ifndef _CYTRON_OPERATOR_H_
#define _CYTRON_OPERATOR_H_

#include <Arduino.h>
#include <CytronMotorDriver.h>
#include "MotorOperator.h"
#include "generated/commands.h"


#define CYTRON_MAX_OUTPUT 255

class CytronOperator : public MotorOperator
{
public:
    CytronOperator();
    CytronOperator(uint8_t pwmPin, uint8_t dirPin, bool inverted = false, bool enabled = false);
    CytronOperator(uint8_t pwm1Pin, uint8_t dirOrPwm2Pin);
    CytronOperator(const CytronOperator& other);
    
    CytronOperator& operator=(const CytronOperator& other);
    bool init() override;
    void setOutput(float percentOutput);
    bool applyConfigUpdate(const Cytron_Config_Data& update);
    
private:
    CytronMD* cytronDriver;
    bool inverted;
    bool enabled;
    uint8_t pwmPin;
    uint8_t dirPin;  // Used for PWM_DIR mode
};

#endif // _CYTRON_OPERATOR_H_