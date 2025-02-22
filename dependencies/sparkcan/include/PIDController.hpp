/**
 * @file PIDController.hpp
 * @brief Header file for the PIDController class for configuring and controlling PID for SPARK motor controllers
 * @author Grayson Arendt
 */

#pragma once
#include "SparkBase.hpp"

/**
 * @class PIDController
 * @brief A class for configuring and controlling PID for SPARK motor controllers
 *
 * This class provides methods for setting and getting various PID parameters such as P, I, D, F gains,
 * and other control parameters specific to Smart Motion and position control.
 */
class PIDController
{
private:
    SparkBase &sparkBase; ///< Reference to the associated SparkBase motor controller

public:
    /**
     * @brief Construct a PIDController from a SparkBase object.
     *
     * @param base The SparkBase object associated with this PIDController.
     */
    explicit PIDController(SparkBase &base);

    /**
     * @brief Set the controller reference value based on the selected control mode.
     *
     * @param controlType CtrlType::kDutyCycle for Duty Cycle, CtrlType::kVelocity for Velocity, CtrlType::kVoltage for Voltage, CtrlType::kPosition for Position
     */
    void SetReference(float setpoint, CtrlType controlType);

    /**
     * @brief Set the feedback device for the PID controller.
     *
     * @param sensor The feedback sensor device.
     */
    void SetFeedbackDevice(uint16_t sensor);

    /**
     * @brief Set the Proportional (P) gain.
     *
     * @param slot The PID slot (0-3).
     * @param p The P gain value.
     */
    void SetP(uint8_t slot, float p);

    /**
     * @brief Set the Integral (I) gain.
     *
     * @param slot The PID slot (0-3).
     * @param i The I gain value.
     */
    void SetI(uint8_t slot, float i);

    /**
     * @brief Set the Derivative (D) gain.
     *
     * @param slot The PID slot (0-3).
     * @param d The D gain value.
     */
    void SetD(uint8_t slot, float d);

    /**
     * @brief Set the Feedforward (F) gain.
     *
     * @param slot The PID slot (0-3).
     * @param f The F gain value.
     */
    void SetF(uint8_t slot, float f);

    /**
     * @brief Set the Integral Zone (I-Zone) for the PID controller.
     *
     * @param slot The PID slot (0-3).
     * @param iZone The I-Zone value.
     */
    void SetIZone(uint8_t slot, float iZone);

    /**
     * @brief Set the Derivative Filter for the PID controller.
     *
     * @param slot The PID slot (0-3).
     * @param dFilter The D-Filter value.
     */
    void SetDFilter(uint8_t slot, float dFilter);

    /**
     * @brief Set the output minimum for the PID controller.
     *
     * @param slot The PID slot (0-3).
     * @param min The minimum output value.
     */
    void SetOutputMin(uint8_t slot, float min);

    /**
     * @brief Set the output maximum for the PID controller.
     *
     * @param slot The PID slot (0-3).
     * @param max The maximum output value.
     */
    void SetOutputMax(uint8_t slot, float max);

    /**
     * @brief Set the Smart Motion maximum velocity.
     *
     * @param slot The Smart Motion slot (0-3).
     * @param maxVel The maximum velocity.
     */
    void SetSmartMotionMaxVelocity(uint8_t slot, float maxVel);

    /**
     * @brief Set the Smart Motion maximum acceleration.
     *
     * @param slot The Smart Motion slot (0-3).
     * @param maxAccel The maximum acceleration.
     */
    void SetSmartMotionMaxAccel(uint8_t slot, float maxAccel);

    /**
     * @brief Set the Smart Motion minimum velocity output.
     *
     * @param slot The Smart Motion slot (0-3).
     * @param minVel The minimum velocity output.
     */
    void SetSmartMotionMinVelOutput(uint8_t slot, float minVel);

    /**
     * @brief Set the allowed closed-loop error for Smart Motion.
     *
     * @param slot The Smart Motion slot (0-3).
     * @param error The allowed closed-loop error.
     */
    void SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error);

    /**
     * @brief Set the acceleration strategy for Smart Motion.
     *
     * @param slot The Smart Motion slot (0-3).
     * @param strategy The acceleration strategy.
     */
    void SetSmartMotionAccelStrategy(uint8_t slot, float strategy);

    /**
     * @brief Set the maximum accumulated integral for the PID controller.
     *
     * @param slot The PID slot (0-3).
     * @param maxAccum The maximum accumulated integral value.
     */
    void SetIMaxAccum(uint8_t slot, float maxAccum);

    /**
     * @brief Set the placeholder parameter in slot 3, position 1.
     *
     * @param slot The PID slot (0-3).
     * @param value The value for the placeholder.
     */
    void SetSlot3Placeholder1(uint8_t slot, float value);

    /**
     * @brief Set the placeholder parameter in slot 3, position 2.
     *
     * @param slot The PID slot (0-3).
     * @param value The value for the placeholder.
     */
    void SetSlot3Placeholder2(uint8_t slot, float value);

    /**
     * @brief Set the placeholder parameter in slot 3, position 3.
     *
     * @param slot The PID slot (0-3).
     * @param value The value for the placeholder.
     */
    void SetSlot3Placeholder3(uint8_t slot, float value);

    /**
     * @brief Enable or disable position PID wrap.
     *
     * @param enable True to enable, false to disable.
     */
    void SetPositionPIDWrapEnable(bool enable);

    /**
     * @brief Set the minimum input for position PID control.
     *
     * @param minInput The minimum input value.
     */
    void SetPositionPIDMinInput(float minInput);

    /**
     * @brief Set the maximum input for position PID control.
     *
     * @param maxInput The maximum input value.
     */
    void SetPositionPIDMaxInput(float maxInput);

    // Get PID Parameters

    /**
     * @brief Get the feedback device ID.
     *
     * @return The feedback device ID.
     */
    uint16_t GetFeedbackDevice();

    /**
     * @brief Get the Proportional (P) gain.
     *
     * @param slot The PID slot (0-3).
     * @return The P gain value.
     */
    float GetP(uint8_t slot);

    /**
     * @brief Get the Integral (I) gain.
     *
     * @param slot The PID slot (0-3).
     * @return The I gain value.
     */
    float GetI(uint8_t slot);

    /**
     * @brief Get the Derivative (D) gain.
     *
     * @param slot The PID slot (0-3).
     * @return The D gain value.
     */
    float GetD(uint8_t slot);

    /**
     * @brief Get the Feedforward (F) gain.
     *
     * @param slot The PID slot (0-3).
     * @return The F gain value.
     */
    float GetF(uint8_t slot);

    /**
     * @brief Get the Integral Zone (I-Zone).
     *
     * @param slot The PID slot (0-3).
     * @return The I-Zone value.
     */
    float GetIZone(uint8_t slot);

    /**
     * @brief Get the Derivative Filter value.
     *
     * @param slot The PID slot (0-3).
     * @return The D-Filter value.
     */
    float GetDFilter(uint8_t slot);

    /**
     * @brief Get the minimum output range.
     *
     * @param slot The PID slot (0-3).
     * @return The minimum output value.
     */
    float GetOutputMin(uint8_t slot);

    /**
     * @brief Get the maximum output range.
     *
     * @param slot The PID slot (0-3).
     * @return The maximum output value.
     */
    float GetOutputMax(uint8_t slot);

    /**
     * @brief Get the Smart Motion maximum velocity.
     *
     * @param slot The Smart Motion slot (0-3).
     * @return The maximum velocity.
     */
    float GetSmartMotionMaxVelocity(uint8_t slot);

    /**
     * @brief Get the Smart Motion maximum acceleration.
     *
     * @param slot The Smart Motion slot (0-3).
     * @return The maximum acceleration.
     */
    float GetSmartMotionMaxAccel(uint8_t slot);

    /**
     * @brief Get the Smart Motion minimum velocity output.
     *
     * @param slot The Smart Motion slot (0-3).
     * @return The minimum velocity output.
     */
    float GetSmartMotionMinVelOutput(uint8_t slot);

    /**
     * @brief Get the allowed closed-loop error for Smart Motion.
     *
     * @param slot The Smart Motion slot (0-3).
     * @return The allowed closed-loop error.
     */
    float GetSmartMotionAllowedClosedLoopError(uint8_t slot);

    /**
     * @brief Get the Smart Motion acceleration strategy.
     *
     * @param slot The Smart Motion slot (0-3).
     * @return The acceleration strategy.
     */
    float GetSmartMotionAccelStrategy(uint8_t slot);

    /**
     * @brief Get the maximum accumulated integral for the PID controller.
     *
     * @param slot The PID slot (0-3).
     * @return The maximum accumulated integral value.
     */
    float GetIMaxAccum(uint8_t slot);

    /**
     * @brief Get the value of slot 3 placeholder 1.
     *
     * @param slot The PID slot (0-3).
     * @return The value of slot 3 placeholder 1.
     */
    float GetSlot3Placeholder1(uint8_t slot);

    /**
     * @brief Get the value of slot 3 placeholder 2.
     *
     * @param slot The PID slot (0-3).
     * @return The value of slot 3 placeholder 2.
     */
    float GetSlot3Placeholder2(uint8_t slot);

    /**
     * @brief Get the value of slot 3 placeholder 3.
     *
     * @param slot The PID slot (0-3).
     * @return The value of slot 3 placeholder 3.
     */
    float GetSlot3Placeholder3(uint8_t slot);

    /**
     * @brief Get whether position PID wrap is enabled.
     *
     * @return True if enabled, false otherwise.
     */
    bool GetPositionPIDWrapEnable();

    /**
     * @brief Get the minimum input for position PID control.
     *
     * @return The minimum input value.
     */
    float GetPositionPIDMinInput();

    /**
     * @brief Get the maximum input for position PID control.
     *
     * @return The maximum input value.
     */
    float GetPositionPIDMaxInput();
};
