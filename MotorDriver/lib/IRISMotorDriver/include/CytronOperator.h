#ifndef _CYTRON_OPERATOR_
#define _CYTRON_OPERATOR_

#include <Arduino.h>

#undef min
#undef max
#include <array>

#include "generated/commands.h"
#include "MotorOperator.h"

#define DEFAULT_CYTRON_MOTOR_NUM 1
#define CYTRON_MAX_OUTPUT 126

class CytronOperator : public MotorOperator
{
  public:
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
    // bool applyConfigUpdate(const Sabertooth_Config_Data& update);

  private:
    // Cytron specific
    // Will need to figure out what goes here
    byte motornum;
};

#endif