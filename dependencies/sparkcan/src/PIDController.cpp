/**
 * @file PIDController.cpp
 * @brief Source file for the PIDController class for configuring and controlling PID for SPARK motor controllers
 * @author Grayson Arendt
 */

#include "PIDController.hpp"

/**
 * @brief Constructor that initializes a PIDController with a reference to a SparkBase object.
 *
 * @param base The SparkBase object associated with this PIDController.
 */
PIDController::PIDController(SparkBase &base)
    : sparkBase(base) {}

void PIDController::SetReference(float setpoint, CtrlType controlType)
{
    switch (static_cast<uint8_t>(controlType))
    {
    case 0:
        sparkBase.SetDutyCycle(setpoint);
        break;
    case 1:
        sparkBase.SetVelocity(setpoint);
        break;
    case 2:
        sparkBase.SetVoltage(setpoint);
        break;
    case 3:
        sparkBase.SetPosition(setpoint);
        break;

    default:
        throw std::invalid_argument("Invalid control type. Must be CtrlType::kDutyCycle (Duty Cycle), CtrlType::kVelocity (Velocity), CtrlType::kVoltage (Voltage), CtrlType::kPosition (Position).");
    }
}

void PIDController::SetFeedbackDevice(uint16_t sensor)
{
    sparkBase.SetFeedbackSensorPID0(sensor);
}

void PIDController::SetP(uint8_t slot, float p)
{
    sparkBase.SetP(slot, p);
}

void PIDController::SetI(uint8_t slot, float i)
{
    sparkBase.SetI(slot, i);
}

void PIDController::SetD(uint8_t slot, float d)
{
    sparkBase.SetD(slot, d);
}

void PIDController::SetF(uint8_t slot, float f)
{
    sparkBase.SetF(slot, f);
}

void PIDController::SetIZone(uint8_t slot, float iZone)
{
    sparkBase.SetIZone(slot, iZone);
}

void PIDController::SetDFilter(uint8_t slot, float dFilter)
{
    sparkBase.SetDFilter(slot, dFilter);
}

void PIDController::SetOutputMin(uint8_t slot, float min)
{
    sparkBase.SetOutputMin(slot, min);
}

void PIDController::SetOutputMax(uint8_t slot, float max)
{
    sparkBase.SetOutputMax(slot, max);
}

void PIDController::SetSmartMotionMaxVelocity(uint8_t slot, float maxVel)
{
    sparkBase.SetSmartMotionMaxVelocity(slot, maxVel);
}

void PIDController::SetSmartMotionMaxAccel(uint8_t slot, float maxAccel)
{
    sparkBase.SetSmartMotionMaxAccel(slot, maxAccel);
}

void PIDController::SetSmartMotionMinVelOutput(uint8_t slot, float minVel)
{
    sparkBase.SetSmartMotionMinVelOutput(slot, minVel);
}

void PIDController::SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error)
{
    sparkBase.SetSmartMotionAllowedClosedLoopError(slot, error);
}

void PIDController::SetSmartMotionAccelStrategy(uint8_t slot, float strategy)
{
    sparkBase.SetSmartMotionAccelStrategy(slot, strategy);
}

void PIDController::SetIMaxAccum(uint8_t slot, float maxAccum)
{
    sparkBase.SetIMaxAccum(slot, maxAccum);
}

void PIDController::SetSlot3Placeholder1(uint8_t slot, float value)
{
    sparkBase.SetSlot3Placeholder1(slot, value);
}

void PIDController::SetSlot3Placeholder2(uint8_t slot, float value)
{
    sparkBase.SetSlot3Placeholder2(slot, value);
}

void PIDController::SetSlot3Placeholder3(uint8_t slot, float value)
{
    sparkBase.SetSlot3Placeholder3(slot, value);
}

void PIDController::SetPositionPIDWrapEnable(bool enable)
{
    sparkBase.SetPositionPIDWrapEnable(enable);
}

void PIDController::SetPositionPIDMinInput(float minInput)
{
    sparkBase.SetPositionPIDMinInput(minInput);
}

void PIDController::SetPositionPIDMaxInput(float maxInput)
{
    sparkBase.SetPositionPIDMaxInput(maxInput);
}

// Get PID Parameters
uint16_t PIDController::GetFeedbackDevice()
{
    return sparkBase.GetFeedbackSensorPID0();
}

float PIDController::GetP(uint8_t slot)
{
    return sparkBase.GetP(slot);
}

float PIDController::GetI(uint8_t slot)
{
    return sparkBase.GetI(slot);
}

float PIDController::GetD(uint8_t slot)
{
    return sparkBase.GetD(slot);
}

float PIDController::GetF(uint8_t slot)
{
    return sparkBase.GetF(slot);
}

float PIDController::GetIZone(uint8_t slot)
{
    return sparkBase.GetIZone(slot);
}

float PIDController::GetDFilter(uint8_t slot)
{
    return sparkBase.GetDFilter(slot);
}

float PIDController::GetOutputMin(uint8_t slot)
{
    return sparkBase.GetOutputMin(slot);
}

float PIDController::GetOutputMax(uint8_t slot)
{
    return sparkBase.GetOutputMax(slot);
}

float PIDController::GetSmartMotionMaxVelocity(uint8_t slot)
{
    return sparkBase.GetSmartMotionMaxVelocity(slot);
}

float PIDController::GetSmartMotionMaxAccel(uint8_t slot)
{
    return sparkBase.GetSmartMotionMaxAccel(slot);
}

float PIDController::GetSmartMotionMinVelOutput(uint8_t slot)
{
    return sparkBase.GetSmartMotionMinVelOutput(slot);
}

float PIDController::GetSmartMotionAllowedClosedLoopError(uint8_t slot)
{
    return sparkBase.GetSmartMotionAllowedClosedLoopError(slot);
}

float PIDController::GetSmartMotionAccelStrategy(uint8_t slot)
{
    return sparkBase.GetSmartMotionAccelStrategy(slot);
}

float PIDController::GetIMaxAccum(uint8_t slot)
{
    return sparkBase.GetIMaxAccum(slot);
}

float PIDController::GetSlot3Placeholder1(uint8_t slot)
{
    return sparkBase.GetSlot3Placeholder1(slot);
}

float PIDController::GetSlot3Placeholder2(uint8_t slot)
{
    return sparkBase.GetSlot3Placeholder2(slot);
}

float PIDController::GetSlot3Placeholder3(uint8_t slot)
{
    return sparkBase.GetSlot3Placeholder3(slot);
}

bool PIDController::GetPositionPIDWrapEnable()
{
    return sparkBase.GetPositionPIDWrapEnable();
}

float PIDController::GetPositionPIDMinInput()
{
    return sparkBase.GetPositionPIDMinInput();
}

float PIDController::GetPositionPIDMaxInput()
{
    return sparkBase.GetPositionPIDMaxInput();
}
