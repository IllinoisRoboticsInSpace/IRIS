#include "MotorOperator.h"

MotorOperator::MotorOperator(bool inverted, bool enabled)
    : inverted(inverted), enabled(enabled)
{
}

MotorOperator::MotorOperator()
    : inverted(false), enabled(false)
{
}

MotorOperator::MotorOperator(const MotorOperator& other)
    : inverted(other.inverted), enabled(other.enabled)
{
}

MotorOperator& MotorOperator::operator=(const MotorOperator& other)
{
    inverted = other.inverted;
    enabled = other.enabled;
    return *this;
}

void MotorOperator::setInverted(bool inverted)
{
    this->inverted = inverted;
}

void MotorOperator::setEnabled(bool enabled)
{
    this->enabled = enabled;
}

bool MotorOperator::getInverted()
{
    return enabled;
}

bool MotorOperator::getEnabled()
{
    return enabled;
}
