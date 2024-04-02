#ifndef _CYTRON_OPERATOR_
#define _CYTRON_OPERATOR_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "generated/commands.h"
#include "MotorOperator.h"

#define DEFAULT_CYTRON_MOTOR_PWM_PIN 2
#define DEFAULT_CYTRON_MOTOR_DIR_PIN 3
#define CYTRON_MAX_OUTPUT 255

class CytronOperator : public MotorOperator
{
  public:
    CytronOperator(byte pin_1, byte pin_2, bool inverted, bool enabled);
    CytronOperator(byte pin_1, byte pin_2);

    CytronOperator();
    CytronOperator(const CytronOperator& other);
    /*!
    Assigns current object to other object
    \param other CytronOperator
    */
    CytronOperator& operator=(const CytronOperator& other);

    //Operation functions
    bool init() override;
    void setOutput(float percentOutput);
    bool applyConfigUpdate(const Cytron_Config_Data& update);
    byte get_pwm_pin() const;
    byte get_dir_pin() const;

  private:
    // Cytron specific
    // Will need to figure out what goes here
    byte pwm_pin; 
    byte dir_pin;
};

#endif