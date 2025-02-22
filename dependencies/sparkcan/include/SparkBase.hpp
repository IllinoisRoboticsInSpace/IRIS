/**
 * @file SparkBase.hpp
 * @brief Header file for the base class for controlling REV Robotics SPARK motor controllers
 * @author Grayson Arendt
 */

#ifndef SPARKBASE_HPP
#define SPARKBASE_HPP

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <variant>
#include <optional>

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define RED "\033[31m"  ///< ANSI escape code for setting terminal text color to red
#define RESET "\033[0m" ///< ANSI escape code for resetting terminal text color to default

constexpr uint8_t PARAM_TYPE_UINT = 0x01;  ///< Parameter type for unsigned integers
constexpr uint8_t PARAM_TYPE_FLOAT = 0x02; ///< Parameter type for floating-point numbers
constexpr uint8_t PARAM_TYPE_BOOL = 0x03;  ///< Parameter type for boolean values

/**
 * @brief System control commands for the SPARK controller
 */

enum class SystemControl : uint32_t
{
    BurnFlash = 0x205FC80,
    FactoryDefaults = 0x2051D00,
    FactoryReset = 0x2051D40,
    Identify = 0x2051D80,
    ResetFaults = 0x2053000,
    ClearStickyFaults = 0x2054400
};

/**
 * @brief Motor control commands for the SPARK controller
 */
enum class MotorControl : uint32_t
{
    Setpoint = 0x2050040,
    DutyCycle = 0x2050080,
    Velocity = 0x2050480,
    SmartVelocity = 0x20504C0,
    Position = 0x2050C80,
    Voltage = 0x2051080,
    Current = 0x20510C0,
    SmartMotion = 0x2051480
};

/**
 * @brief Status periods for the SPARK controller
 */
enum class Status : uint32_t
{
    Period0 = 0x2051800,
    Period1 = 0x2051840,
    Period2 = 0x2051880,
    Period3 = 0x20518C0,
    Period4 = 0x2051900
};

/**
 * @brief Parameters for the SPARK controller
 */
enum class Parameter : uint32_t
{
    kInputMode = 1,
    kMotorType = 2,
    kCommAdvance = 3,
    kSensorType = 4,
    kCtrlType = 5,
    kIdleMode = 6,
    kInputDeadband = 7,
    kFeedbackSensorPID0 = 8,
    kFeedbackSensorPID1 = 9,
    kPolePairs = 10,
    kCurrentChop = 11,
    kCurrentChopCycles = 12,
    kP_0 = 13,
    kI_0 = 14,
    kD_0 = 15,
    kF_0 = 16,
    kIZone_0 = 17,
    kDFilter_0 = 18,
    kOutputMin_0 = 19,
    kOutputMax_0 = 20,
    kP_1 = 21,
    kI_1 = 22,
    kD_1 = 23,
    kF_1 = 24,
    kIZone_1 = 25,
    kDFilter_1 = 26,
    kOutputMin_1 = 27,
    kOutputMax_1 = 28,
    kP_2 = 29,
    kI_2 = 30,
    kD_2 = 31,
    kF_2 = 32,
    kIZone_2 = 33,
    kDFilter_2 = 34,
    kOutputMin_2 = 35,
    kOutputMax_2 = 36,
    kP_3 = 37,
    kI_3 = 38,
    kD_3 = 39,
    kF_3 = 40,
    kIZone_3 = 41,
    kDFilter_3 = 42,
    kOutputMin_3 = 43,
    kOutputMax_3 = 44,
    kInverted = 45,
    kOutputRatio = 46,
    kSerialNumberLow = 47,
    kSerialNumberMid = 48,
    kSerialNumberHigh = 49,
    kLimitSwitchFwdPolarity = 50,
    kLimitSwitchRevPolarity = 51,
    kHardLimitFwdEn = 52,
    kHardLimitRevEn = 53,
    kSoftLimitFwdEn = 54,
    kSoftLimitRevEn = 55,
    kRampRate = 56,
    kFollowerID = 57,
    kFollowerConfig = 58,
    kSmartCurrentStallLimit = 59,
    kSmartCurrentFreeLimit = 60,
    kSmartCurrentConfig = 61,
    kMotorKv = 63,
    kMotorR = 64,
    kMotorL = 65,
    kEncoderCountsPerRev = 69,
    kEncoderAverageDepth = 70,
    kEncoderSampleDelta = 71,
    kEncoderInverted = 72,
    kClosedLoopVoltageMode = 74,
    kCompensatedNominalVoltage = 75,
    kSmartMotionMaxVelocity_0 = 76,
    kSmartMotionMaxAccel_0 = 77,
    kSmartMotionMinVelOutput_0 = 78,
    kSmartMotionAllowedClosedLoopError_0 = 79,
    kSmartMotionAccelStrategy_0 = 80,
    kSmartMotionMaxVelocity_1 = 81,
    kSmartMotionMaxAccel_1 = 82,
    kSmartMotionMinVelOutput_1 = 83,
    kSmartMotionAllowedClosedLoopError_1 = 84,
    kSmartMotionAccelStrategy_1 = 85,
    kSmartMotionMaxVelocity_2 = 86,
    kSmartMotionMaxAccel_2 = 87,
    kSmartMotionMinVelOutput_2 = 88,
    kSmartMotionAllowedClosedLoopError_2 = 89,
    kSmartMotionAccelStrategy_2 = 90,
    kSmartMotionMaxVelocity_3 = 91,
    kSmartMotionMaxAccel_3 = 92,
    kSmartMotionMinVelOutput_3 = 93,
    kSmartMotionAllowedClosedLoopError_3 = 94,
    kSmartMotionAccelStrategy_3 = 95,
    kIMaxAccum_0 = 96,
    kSlot3Placeholder1_0 = 97,
    kSlot3Placeholder2_0 = 98,
    kSlot3Placeholder3_0 = 99,
    kIMaxAccum_1 = 100,
    kSlot3Placeholder1_1 = 101,
    kSlot3Placeholder2_1 = 102,
    kSlot3Placeholder3_1 = 103,
    kIMaxAccum_2 = 104,
    kSlot3Placeholder1_2 = 105,
    kSlot3Placeholder2_2 = 106,
    kSlot3Placeholder3_2 = 107,
    kIMaxAccum_3 = 108,
    kSlot3Placeholder1_3 = 109,
    kSlot3Placeholder2_3 = 110,
    kSlot3Placeholder3_3 = 111,
    kPositionConversionFactor = 112,
    kVelocityConversionFactor = 113,
    kClosedLoopRampRate = 114,
    kSoftLimitFwd = 115,
    kSoftLimitRev = 116,
    kAnalogPositionConversion = 119,
    kAnalogVelocityConversion = 120,
    kAnalogAverageDepth = 121,
    kAnalogSensorMode = 122,
    kAnalogInverted = 123,
    kAnalogSampleDelta = 124,
    kDataPortConfig = 127,
    kAltEncoderCountsPerRev = 128,
    kAltEncoderAverageDepth = 129,
    kAltEncoderSampleDelta = 130,
    kAltEncoderInverted = 131,
    kAltEncoderPositionFactor = 132,
    kAltEncoderVelocityFactor = 133,
    kHallSensorSampleRate = 136,
    kHallSensorAverageDepth = 137,
    kDutyCyclePositionFactor = 139,
    kDutyCycleVelocityFactor = 140,
    kDutyCycleInverted = 141,
    kDutyCycleAverageDepth = 143,
    kPositionPIDWrapEnable = 149,
    kPositionPIDMinInput = 150,
    kPositionPIDMaxInput = 151,
    kDutyCyclePrescalar = 153,
    kDutyCycleZeroOffset = 154
};

/**
 * @brief Motor type parameter
 */
enum class MotorType : uint8_t
{
    kBrushed = 0,
    kBrushless = 1
};

/**
 * @brief Sensor type parameter
 */
enum class SensorType : uint8_t
{
    kNoSensor = 0,
    kHallSensor = 1,
    kEncoder = 2
};

/**
 * @brief Control type parameter
 */
enum class CtrlType : uint8_t
{
    kDutyCycle = 0,
    kVelocity = 1,
    kVoltage = 2,
    kPosition = 3
};

/**
 * @brief Idle mode parameter
 */
enum class IdleMode : uint8_t
{
    kCoast = 0,
    kBrake = 1
};

/**
 * @class SparkBase
 * @brief A base class for controlling REV Robotics SPARK motor controllers via CAN bus
 *
 * This class provides methods to configure, control, and monitor SPARK motor controllers.
 * It supports various control modes, parameter settings, and status readings.
 */
class SparkBase
{
private:
    static int soc;                   ///< Socket descriptor for CAN communication
    std::string interfaceName; ///< Name of the CAN interface
    uint8_t deviceId;          ///< Device ID for the SPARK controller on the CAN bus
    struct sockaddr_can addr;  ///< Socket address for the CAN interface
    struct ifreq ifr;          ///< Interface request structure for CAN operations
    mutable std::map<Status,
                     std::pair<uint64_t, std::chrono::steady_clock::time_point>>
        cachedStatus; ///< Cache for periodic status data

    /**
     * @brief Sends a CAN frame
     *
     * @param arbitrationId The arbitration ID of the CAN frame
     * @param dlc The data length code (number of bytes in the data array)
     * @param data The data payload to send in the CAN frame
     */
    void SendCanFrame(
        uint32_t arbitrationId, uint8_t dlc,
        const std::array<uint8_t, 8> &data = std::array<uint8_t, 8>{}) const;

    /**
     * @brief Sends a control message to the SPARK controller
     *
     * @param command The control command to send (either MotorControl or SystemControl)
     * @param commandName The control command's name
     * @param value The value associated with the control command
     * @param minValue The minimum allowed value for the command (optional)
     * @param maxValue The maximum allowed value for the command (optional)
     *
     * @throws std::invalid_argument If the command value is not finite
     * @throws std::out_of_range If the value is outside the specified range (will default to min and max of datatype when not provided)
     */
    void SendControlMessage(
        std::variant<MotorControl, SystemControl> command, std::string commandName, float value, std::optional<float> minValue = std::nullopt,
        std::optional<float> maxValue = std::nullopt) const;

    /**
     * @brief Reads periodic status data from the SPARK controller
     *
     * @param period The status period to read from
     * @return uint64_t The status data read from the controller
     */
    uint64_t ReadPeriodicStatus(Status period) const;

    /**
     * @brief Sets a parameter on the SPARK controller
     *
     * @param parameterId The ID of the parameter to set
     * @param parameterType The type of the parameter (e.g., UINT, FLOAT, BOOL)
     * @param parameterName The name of the parameter
     * @param value The value to set the parameter to (can be float, uint32_t, uint16_t, uint8_t, or bool)
     * @param minValue The minimum allowed value for the parameter (optional)
     * @param maxValue The maximum allowed value for the parameter (optional)
     * @param customErrorMessage A custom error message to use if the value is out of range (optional)
     *
     * @throws std::invalid_argument If the parameter type is invalid or if a float value is not finite
     * @throws std::out_of_range If the value is outside the specified range (will default to min and max of datatype when not provided)
     */
    void SetParameter(
        Parameter parameterId,
        uint8_t parameterType,
        std::string parameterName,
        std::variant<float, uint32_t, uint16_t, uint8_t, bool> value,
        std::optional<float> minValue = std::nullopt,
        std::optional<float> maxValue = std::nullopt,
        std::optional<std::string> customErrorMessage = std::nullopt);

    /**
     * @brief Reads the value of a specified parameter from the device via CAN communication.
     *
     * @param parameterId The ID of the parameter to read (enumerated by the Parameter type).
     * @return std::variant<float, uint32_t, bool> The value of the parameter, which can be a float, uint32_t, or bool.
     * @throws std::runtime_error If the CAN message response is not valid or if the parameter cannot be read.
     */
    std::variant<float, uint32_t, bool> ReadParameter(Parameter parameterId);

public:
    /**
     * @brief Initializes SparkBase with the specified CAN interface and ID
     *
     * @param interfaceName The name of the CAN interface (e.g., "can0")
     * @param deviceId The CAN ID of the SPARK controller (0-62)
     * @throws std::out_of_range if deviceId is greater than 62
     * @throws std::system_error if socket creation fails, with detailed error information
     * @throws std::runtime_error if IOCTL fails or binding to the interface fails, with detailed error information
     *
     * @details This constructor attempts to initialize the CAN bus connection. If it fails, it will throw
     * an exception with a detailed error message that includes possible causes and suggested solutions.
     * Common issues that may cause exceptions include:
     * - Invalid device ID
     * - CAN modules not loaded
     * - System resource limitations
     * - CAN interface not existing
     * - CAN interface not being up
     * - CAN bus not initialized
     * - Interface already bound to another program
     */
    SparkBase(const std::string &interfaceName, uint8_t deviceId);

    /**
     * @brief Destructor for SparkBase
     *
     * Closes the socket connection
     */
    virtual ~SparkBase();

    // SystemControl Methods //

    /**
     * @brief Sends a heartbeat signal to keep all SPARK controllers active
     */
    static void Heartbeat();

    /**
     * @brief Resets all faults on the SPARK controller
     */
    void ResetFaults();

    /**
     * @brief Clears sticky faults on the SPARK controller
     */
    void ClearStickyFaults();

    /**
     * @brief Burns the current configuration to the SPARK controller's flash memory
     */
    void BurnFlash();

    /**
     * @brief Resets the SPARK controller to factory default settings
     */
    void FactoryDefaults();

    /**
     * @brief Performs a factory reset on the SPARK controller
     */
    void FactoryReset();

    /**
     * @brief Triggers the SPARK controller to identify itself
     */
    void Identify();

    // MotorControl Methods //

    /**
     * @brief Sets the value for the currently set control type
     * @param setpoint The desired value
     */
    void SetSetpoint(float setpoint);

    /**
     * @brief Sets the motor's applied output
     * @param dutyCycle The desired applied output, range: [-1.0, 1.0]
     */
    void SetDutyCycle(float dutyCycle);

    /**
     * @brief Sets the motor's velocity
     * @param velocity The desired velocity
     */
    void SetVelocity(float velocity);

    /**
     * @brief Sets the motor's smart velocity
     * @param smartVelocity The desired smart velocity
     */
    void SetSmartVelocity(float smartVelocity);

    /**
     * @brief Sets the motor's position
     * @param position The desired position
     */
    void SetPosition(float position);

    /**
     * @brief Sets the motor's voltage
     * @param voltage The desired voltage
     */
    void SetVoltage(float voltage);

    /**
     * @brief Sets the motor's current
     * @param current The desired current
     */
    void SetCurrent(float current);

    /**
     * @brief Sets the motor's smart motion
     * @param smartMotion The desired smart motion value
     */
    void SetSmartMotion(float smartMotion);

    // Status Methods //

    /**
     * @brief Retrieves the current applied output
     * @return float The applied output, range: [-1.0, 1.0]
     */
    float GetDutyCycle() const;

    /**
     * @brief Retrieves the current faults
     * @return uint16_t A bitfield representing the current faults
     */
    uint16_t GetFaults() const;

    /**
     * @brief Retrieves the sticky faults
     * @return uint16_t A bitfield representing the sticky faults
     */
    uint16_t GetStickyFaults() const;

    /**
     * @brief Checks if the motor is inverted
     * @return bool True if the motor is inverted, false otherwise
     */
    bool GetInverted() const;

    /**
     * @brief Gets the current idle mode
     * @return bool True if in brake mode, false if in coast mode
     */
    bool GetIdleMode() const;

    /**
     * @brief Checks if the SPARK controller is in follower mode
     * @return bool True if in follower mode, false otherwise
     */
    bool IsFollower() const;

    /**
     * @brief Gets the current velocity
     * @return float The current velocity in RPM
     */
    float GetVelocity() const;

    /**
     * @brief Gets the current temperature of the SPARK controller
     * @return float The temperature in degrees Celsius
     */
    float GetTemperature() const;

    /**
     * @brief Gets the current voltage of the SPARK controller
     * @return float The voltage in volts
     */
    float GetVoltage() const;

    /**
     * @brief Gets the current drawn by the SPARK controller
     * @return float The current in amperes
     */
    float GetCurrent() const;

    /**
     * @brief Gets the current position of the motor
     * @return float The position in ticks
     */
    float GetPosition() const;

    /**
     * @brief Gets the current analog voltage
     * @return float The analog voltage in volts
     */
    float GetAnalogVoltage() const;

    /**
     * @brief Gets the current analog velocity
     * @return float The analog velocity in RPM
     */
    float GetAnalogVelocity() const;

    /**
     * @brief Gets the current analog position
     * @return float The analog position in ticks
     */
    float GetAnalogPosition() const;

    /**
     * @brief Gets the velocity from the alternate encoder
     * @return float The alternate encoder velocity in RPM
     */
    float GetAlternateEncoderVelocity() const;

    /**
     * @brief Gets the position from the alternate encoder
     * @return float The alternate encoder position in ticks
     */
    float GetAlternateEncoderPosition() const;

    // Parameter Setters //

    /**
     * @brief Sets the input mode
     * @param mode The input mode
     */
    void SetInputMode(uint8_t mode);

    /**
     * @brief Sets the motor type
     * @param type MotorType::kBrushed for Brushed, MotorType::kBrushless for Brushless
     */
    void SetMotorType(MotorType type);

    /**
     * @brief Sets the sensor type
     * @param sensor SensorType::kNoSensor for No Sensor, SensorType::kHallSensor for Hall Sensor, SensorType::kEncoder for Encoder
     */
    void SetSensorType(SensorType sensor);

    /**
     * @brief Sets the idle mode
     * @param mode IdleMode::kCoast for Coast, IdleMode::kBrake for Brake
     */
    void SetIdleMode(IdleMode mode);

    /**
     * @brief Sets the input deadband
     *
     * @param deadband The deadband value
     */
    void SetInputDeadband(float deadband);

    /**
     * @brief Sets whether the motor is inverted
     * @param inverted True to invert the motor, false otherwise
     */
    void SetInverted(bool inverted);

    /**
     * @brief Sets the ramp rate
     * @param rate The ramp rate in seconds from neutral to full output
     */
    void SetRampRate(float rate);

    // Advanced //

    /**
     * @brief Sets the motor Kv (velocity constant)
     * @param kv The Kv value
     */
    void SetMotorKv(uint16_t kv);

    /**
     * @brief Sets the motor resistance
     * @param r The resistance value
     */
    void SetMotorR(uint16_t r);

    /**
     * @brief Sets the motor inductance
     * @param l The inductance value
     */
    void SetMotorL(uint16_t l);

    // Closed Loop //

    /**
     * @brief Sets the control type
     * @param type CtrlType::kDutyCycle for Duty Cycle, CtrlType::kVelocity for Velocity, CtrlType::kVoltage for Voltage, CtrlType::kPosition for Position
     */
    void SetCtrlType(CtrlType type);

    /**
     * @brief Sets the feedback sensor for PID0
     * @param sensor The sensor type
     */
    void SetFeedbackSensorPID0(uint16_t sensor);

    /**
     * @brief Sets the closed loop voltage mode
     * @param mode 0 for Disabled, 1 for Control Loop Voltage Output Mode, 2 for Voltage Compensation Mode

     */
    void SetClosedLoopVoltageMode(uint8_t mode);

    /**
     * @brief Sets the compensated nominal voltage
     * @param voltage The nominal voltage to compensate for
     */
    void SetCompensatedNominalVoltage(float voltage);

    /**
     * @brief Enables or disables position PID wrap
     * @param enable True to enable, false to disable
     */
    void SetPositionPIDWrapEnable(bool enable);

    /**
     * @brief Sets the minimum input for position PID
     * @param minInput The minimum input value
     */
    void SetPositionPIDMinInput(float minInput);

    /**
     * @brief Sets the maximum input for position PID
     * @param maxInput The maximum input value
     * @throws std::invalid_argument if maxInput is not a finite number
     */
    void SetPositionPIDMaxInput(float maxInput);

    // Brushless //

    /**
     * @brief Sets the number of pole pairs for brushless motors
     * @param pairs The number of pole pairs
     */
    void SetPolePairs(uint16_t pairs);

    // Current Limit //

    /**
     * @brief Sets the current chop limit
     * @param chop The current chop limit (0-125 amps)
     */
    void SetCurrentChop(float chop);

    /**
     * @brief Sets the number of cycles for current chopping
     * @param cycles The number of cycles
     */
    void SetCurrentChopCycles(uint16_t cycles);

    /**
     * @brief Sets the smart current stall limit
     * @param limit The stall current limit
     */
    void SetSmartCurrentStallLimit(uint16_t limit);

    /**
     * @brief Sets the smart current free limit
     * @param limit The free current limit
     */
    void SetSmartCurrentFreeLimit(uint16_t limit);

    /**
     * @brief Sets the smart current configuration
     * @param config The configuration value
     */
    void SetSmartCurrentConfig(uint16_t config);

    // PIDF //

    /**
     * @brief Sets the proportional gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param p The proportional gain value
     */
    void SetP(uint8_t slot, float p);

    /**
     * @brief Sets the integral gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param i The integral gain value
     * @throws std::out_of_range if slot is greater than 3
     */
    void SetI(uint8_t slot, float i);

    /**
     * @brief Sets the derivative gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param d The derivative gain value
     * @throws std::out_of_range if slot is greater than 3
     */
    void SetD(uint8_t slot, float d);

    /**
     * @brief Sets the feedforward gain for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param f The feedforward gain value
     * @throws std::out_of_range if slot is greater than 3
     */
    void SetF(uint8_t slot, float f);

    /**
     * @brief Sets the integral zone for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param iZone The integral zone value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetIZone(uint8_t slot, float iZone);

    /**
     * @brief Sets the derivative filter for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param dFilter The derivative filter value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetDFilter(uint8_t slot, float dFilter);

    /**
     * @brief Sets the output minimum for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param min The minimum output value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetOutputMin(uint8_t slot, float min);

    /**
     * @brief Sets the output maximum for the specified PID slot
     * @param slot The PID slot (0-3)
     * @param max The maximum output value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetOutputMax(uint8_t slot, float max);

    // Limits //

    /**
     * @brief Enables or disables the forward hard limit switch
     * @param enable True to enable, false to disable
     */
    void SetHardLimitFwdEn(bool enable);

    /**
     * @brief Enables or disables the reverse hard limit switch
     * @param enable True to enable, false to disable
     */
    void SetHardLimitRevEn(bool enable);

    /**
     * @brief Sets the polarity of the forward limit switch
     * @param polarity True for normally open, false for normally closed
     */
    void SetLimitSwitchFwdPolarity(bool polarity);

    /**
     * @brief Sets the polarity of the reverse limit switch
     * @param polarity True for normally open, false for normally closed
     */
    void SetLimitSwitchRevPolarity(bool polarity);

    /**
     * @brief Enables or disables the forward soft limit
     * @param enable True to enable, false to disable
     */
    void SetSoftLimitFwdEn(bool enable);

    /**
     * @brief Enables or disables the reverse soft limit
     * @param enable True to enable, false to disable
     */
    void SetSoftLimitRevEn(bool enable);

    /**
     * @brief Sets the forward soft limit
     * @param limit The forward soft limit value
     */
    void SetSoftLimitFwd(float limit);

    /**
     * @brief Sets the reverse soft limit
     * @param limit The reverse soft limit value
     */
    void SetSoftLimitRev(float limit);

    // Follower //

    /**
     * @brief Sets the follower ID for this SparkBase
     * @param id The CAN ID of the SPARK controller to follow
     */
    void SetFollowerID(uint32_t id);

    /**
     * @brief Sets the follower configuration
     * @param config The follower configuration value
     */
    void SetFollowerConfig(uint32_t config);

    // Encoder Port //

    /**
     * @brief Sets the encoder counts per revolution
     * @param counts The number of counts per revolution
     */
    void SetEncoderCountsPerRev(uint16_t counts);

    /**
     * @brief Sets the encoder average depth
     * @param depth The average depth (1-64)
     */
    void SetEncoderAverageDepth(uint8_t depth);

    /**
     * @brief Sets the encoder sample delta
     * @param delta The sample delta (1-255)
     */
    void SetEncoderSampleDelta(uint8_t delta);

    /**
     * @brief Sets whether the encoder is inverted
     * @param inverted True to invert the encoder, false otherwise
     */
    void SetEncoderInverted(bool inverted);

    /**
     * @brief Sets the position conversion factor
     * @param factor The position conversion factor
     */
    void SetPositionConversionFactor(float factor);

    /**
     * @brief Sets the velocity conversion factor
     * @param factor The velocity conversion factor
     */
    void SetVelocityConversionFactor(float factor);

    /**
     * @brief Sets the closed loop ramp rate
     * @param rampRate The ramp rate in seconds from neutral to full output
     */
    void SetClosedLoopRampRate(float rampRate);

    /**
     * @brief Sets the hall sensor sample rate
     * @param rate The sample rate in Hz
     */
    void SetHallSensorSampleRate(float rate);

    /**
     * @brief Sets the hall sensor average depth
     * @param depth The average depth
     */
    void SetHallSensorAverageDepth(uint16_t depth);

    // Smart Motion //

    /**
     * @brief Sets the maximum velocity for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param maxVel The maximum velocity
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionMaxVelocity(uint8_t slot, float maxVel);

    /**
     * @brief Sets the maximum acceleration for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param maxAccel The maximum acceleration
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionMaxAccel(uint8_t slot, float maxAccel);

    /**
     * @brief Sets the minimum velocity output for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param minVel The minimum velocity
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionMinVelOutput(uint8_t slot, float minVel);

    /**
     * @brief Sets the allowed closed loop error for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param error The allowed closed loop error
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error);

    /**
     * @brief Sets the acceleration strategy for Smart Motion in the specified slot
     * @param slot The Smart Motion slot (0-3)
     * @param strategy The acceleration strategy
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSmartMotionAccelStrategy(uint8_t slot, float strategy);

    /**
     * @brief Sets the maximum accumulator value for the I term in the specified slot
     * @param slot The PID slot (0-3)
     * @param maxAccum The maximum accumulator value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetIMaxAccum(uint8_t slot, float maxAccum);

    /**
     * @brief Sets a placeholder value for slot 3 (purpose undefined)
     * @param slot The PID slot (0-3)
     * @param value The placeholder value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSlot3Placeholder1(uint8_t slot, float value);

    /**
     * @brief Sets a placeholder value for slot 3 (purpose undefined)
     * @param slot The PID slot (0-3)
     * @param value The placeholder value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSlot3Placeholder2(uint8_t slot, float value);

    /**
     * @brief Sets a placeholder value for slot 3 (purpose undefined)
     * @param slot The PID slot (0-3)
     * @param value The placeholder value
     * @throws std::invalid_argument if slot is greater than 3
     */
    void SetSlot3Placeholder3(uint8_t slot, float value);

    // Analog Sensor //

    /**
     * @brief Sets the conversion factor for analog position readings
     * @param factor The conversion factor to apply to raw analog readings
     */
    void SetAnalogPositionConversion(float factor);

    /**
     * @brief Sets the conversion factor for analog velocity readings
     * @param factor The conversion factor to apply to raw analog readings
     */
    void SetAnalogVelocityConversion(float factor);

    /**
     * @brief Sets the average depth for analog readings
     * @param depth The average depth
     */
    void SetAnalogAverageDepth(uint16_t depth);

    /**
     * @brief Sets the analog sensor mode
     * @param mode 0 for Absolute, 1 for Relative
     */
    void SetAnalogSensorMode(uint8_t mode);

    /**
     * @brief Sets whether the analog sensor is inverted
     * @param inverted True to invert the sensor, false otherwise
     */
    void SetAnalogInverted(bool inverted);

    /**
     * @brief Sets the sample delta for analog readings
     * @param delta The sample delta
     */
    void SetAnalogSampleDelta(uint16_t delta);

    // Alternate Encoder  //

    /**
     * @brief Configures the data port
     * @param config 0 for Default, 1 for Alternate Encoder Mode
     */
    void SetDataPortConfig(uint8_t config);

    /**
     * @brief Sets the counts per revolution for the alternate encoder
     * @param counts The number of counts per revolution
     */
    void SetAltEncoderCountsPerRev(uint16_t counts);

    /**
     * @brief Sets the average depth for the alternate encoder
     * @param depth The average depth (1-64)
     */
    void SetAltEncoderAverageDepth(uint8_t depth);

    /**
     * @brief Sets the sample delta for the alternate encoder
     * @param delta The sample delta (1-255)
     */
    void SetAltEncoderSampleDelta(uint8_t delta);

    /**
     * @brief Sets whether the alternate encoder is inverted
     * @param inverted True to invert the encoder, false otherwise
     */
    void SetAltEncoderInverted(bool inverted);

    /**
     * @brief Sets the position factor for the alternate encoder
     * @param factor The position factor
     */
    void SetAltEncoderPositionFactor(float factor);

    /**
     * @brief Sets the velocity factor for the alternate encoder
     * @param factor The velocity factor
     */
    void SetAltEncoderVelocityFactor(float factor);

    // Duty Cycle Absolute Encoder //

    /**
     * @brief Sets the position factor for the duty cycle encoder
     * @param factor The position factor
     */
    void SetDutyCyclePositionFactor(float factor);

    /**
     * @brief Sets the velocity factor for the duty cycle encoder
     * @param factor The velocity factor
     */
    void SetDutyCycleVelocityFactor(float factor);

    /**
     * @brief Sets whether the duty cycle encoder is inverted
     * @param inverted True to invert the encoder, false otherwise
     */
    void SetDutyCycleInverted(bool inverted);

    /**
     * @brief Sets the average depth for the duty cycle encoder
     * @param depth The average depth (0-7)
     */
    void SetDutyCycleAverageDepth(uint8_t depth);

    /**
     * @brief Sets the prescalar for the duty cycle encoder
     * @param prescalar The prescalar value (0-71)
     */
    void SetDutyCyclePrescalar(uint8_t prescalar);

    /**
     * @brief Sets the zero offset for the duty cycle encoder
     * @param offset The zero offset (0-1)
     */
    void SetDutyCycleZeroOffset(float offset);

    // Parameter Getters //

    // Basic //
    /**
     * @brief Get the motor type.
     * @return Motor type as uint8_t. (0 = Brushed, 1 = Brushless)
     */
    uint8_t GetMotorType();

    /**
     * @brief Get the sensor type.
     * @return Sensor type as uint8_t. (0 = No Sensor, 1 = Hall Sensor, 2 = Encoder)
     */
    uint8_t GetSensorType();

    /**
     * @brief Get the idle mode.
     * @return Idle mode as uint8_t. (0 = Coast, 1 = Brake)
     */
    uint8_t GetIdleMode();

    /**
     * @brief Get the input deadband.
     * @return Input deadband as float.
     */
    float GetInputDeadband();

    /**
     * @brief Get whether the motor is inverted.
     * @return Inverted state as bool.
     */
    bool GetInverted();

    /**
     * @brief Get the ramp rate.
     * @return Ramp rate as float.
     */
    float GetRampRate();

    // Advanced //

    /**
     * @brief Get the motor Kv rating.
     * @return Motor Kv as uint16_t.
     */
    uint16_t GetMotorKv();

    /**
     * @brief Get the motor resistance.
     * @return Motor resistance as uint16_t.
     */
    uint16_t GetMotorR();

    /**
     * @brief Get the motor inductance.
     * @return Motor inductance as uint16_t.
     */
    uint16_t GetMotorL();

    // Closed Loop //

    /**
     * @brief Get the control type.
     * @return Control type as uint8_t. (0 = Duty Cycle, 1 = Velocity, 2 = Voltage, 3 = Position)
     */
    uint8_t GetCtrlType();

    /**
     * @brief Get the feedback sensor PID0 value.
     * @return Feedback sensor PID0 as uint16_t.
     */
    uint16_t GetFeedbackSensorPID0();

    /**
     * @brief Get the closed loop voltage mode.
     * @return Closed loop voltage mode as uint8_t.
     */
    uint8_t GetClosedLoopVoltageMode();

    /**
     * @brief Get the compensated nominal voltage.
     * @return Compensated nominal voltage as float.
     */
    float GetCompensatedNominalVoltage();

    /**
     * @brief Get the position PID wrap enable state.
     * @return Position PID wrap enable state as bool.
     */
    bool GetPositionPIDWrapEnable();

    /**
     * @brief Get the position PID minimum input.
     * @return Position PID minimum input as float.
     */
    float GetPositionPIDMinInput();

    /**
     * @brief Get the position PID maximum input.
     * @return Position PID maximum input as float.
     */
    float GetPositionPIDMaxInput();

    // Brushless //

    /**
     * @brief Get the number of pole pairs.
     * @return Pole pairs as uint16_t.
     */
    uint16_t GetPolePairs();

    // Current Limit //

    /**
     * @brief Get the current chop value.
     * @return Current chop as float.
     */
    float GetCurrentChop();

    /**
     * @brief Get the current chop cycles.
     * @return Current chop cycles as uint16_t.
     */
    uint16_t GetCurrentChopCycles();

    /**
     * @brief Get the smart current stall limit.
     * @return Smart current stall limit as uint16_t.
     */
    uint16_t GetSmartCurrentStallLimit();

    /**
     * @brief Get the smart current free limit.
     * @return Smart current free limit as uint16_t.
     */
    uint16_t GetSmartCurrentFreeLimit();

    /**
     * @brief Get the smart current configuration.
     * @return Smart current config as uint16_t.
     */
    uint16_t GetSmartCurrentConfig();

    // PIDF //

    /**
     * @brief Get the proportional (P) constant for a given slot.
     * @param slot The PID slot (0-3).
     * @return Proportional constant as float.
     */
    float GetP(uint8_t slot);

    /**
     * @brief Get the integral (I) constant for a given slot.
     * @param slot The PID slot (0-3).
     * @return Integral constant as float.
     */
    float GetI(uint8_t slot);

    /**
     * @brief Get the derivative (D) constant for a given slot.
     * @param slot The PID slot (0-3).
     * @return Derivative constant as float.
     */
    float GetD(uint8_t slot);

    /**
     * @brief Get the feedforward (F) constant for a given slot.
     * @param slot The PID slot (0-3).
     * @return Feedforward constant as float.
     */
    float GetF(uint8_t slot);

    /**
     * @brief Get the IZone value for a given slot.
     * @param slot The PID slot (0-3).
     * @return IZone value as float.
     */
    float GetIZone(uint8_t slot);

    /**
     * @brief Get the DFilter value for a given slot.
     * @param slot The PID slot (0-3).
     * @return DFilter value as float.
     */
    float GetDFilter(uint8_t slot);

    /**
     * @brief Get the output minimum value for a given slot.
     * @param slot The PID slot (0-3).
     * @return Output minimum value as float.
     */
    float GetOutputMin(uint8_t slot);

    /**
     * @brief Get the output maximum value for a given slot.
     * @param slot The PID slot (0-3).
     * @return Output maximum value as float.
     */
    float GetOutputMax(uint8_t slot);

    // Limits //

    /**
     * @brief Get the forward hard limit enable state.
     * @return Forward hard limit enable state as bool.
     */
    bool GetHardLimitFwdEn();

    /**
     * @brief Get the reverse hard limit enable state.
     * @return Reverse hard limit enable state as bool.
     */
    bool GetHardLimitRevEn();

    /**
     * @brief Get the forward limit switch polarity.
     * @return Forward limit switch polarity as bool.
     */
    bool GetLimitSwitchFwdPolarity();

    /**
     * @brief Get the reverse limit switch polarity.
     * @return Reverse limit switch polarity as bool.
     */
    bool GetLimitSwitchRevPolarity();

    /**
     * @brief Get the forward soft limit enable state.
     * @return Forward soft limit enable state as bool.
     */
    bool GetSoftLimitFwdEn();

    /**
     * @brief Get the reverse soft limit enable state.
     * @return Reverse soft limit enable state as bool.
     */
    bool GetSoftLimitRevEn();

    /**
     * @brief Get the forward soft limit value.
     * @return Forward soft limit as float.
     */
    float GetSoftLimitFwd();

    /**
     * @brief Get the reverse soft limit value.
     * @return Reverse soft limit as float.
     */
    float GetSoftLimitRev();

    // Follower //

    /**
     * @brief Get the follower ID.
     * @return Follower ID as uint32_t.
     */
    uint32_t GetFollowerID();

    /**
     * @brief Get the follower configuration.
     * @return Follower configuration as uint32_t.
     */
    uint32_t GetFollowerConfig();

    // Encoder Port //

    /**
     * @brief Get the encoder counts per revolution.
     * @return Encoder counts per revolution as uint16_t.
     */
    uint16_t GetEncoderCountsPerRev();

    /**
     * @brief Get the encoder average depth.
     * @return Encoder average depth as uint8_t.
     */
    uint8_t GetEncoderAverageDepth();

    /**
     * @brief Get the encoder sample delta.
     * @return Encoder sample delta as uint8_t.
     */
    uint8_t GetEncoderSampleDelta();

    /**
     * @brief Get the encoder inversion state.
     * @return Encoder inversion state as bool.
     */
    bool GetEncoderInverted();

    /**
     * @brief Get the position conversion factor.
     * @return Position conversion factor as float.
     */
    float GetPositionConversionFactor();

    /**
     * @brief Get the velocity conversion factor.
     * @return Velocity conversion factor as float.
     */
    float GetVelocityConversionFactor();

    /**
     * @brief Get the closed loop ramp rate.
     * @return Closed loop ramp rate as float.
     */
    float GetClosedLoopRampRate();

    /**
     * @brief Get the Hall sensor sample rate.
     * @return Hall sensor sample rate as float.
     */
    float GetHallSensorSampleRate();

    /**
     * @brief Get the Hall sensor average depth.
     * @return Hall sensor average depth as uint16_t.
     */
    uint16_t GetHallSensorAverageDepth();

    // Smart Motion //

    /**
     * @brief Get the maximum velocity for Smart Motion in a given slot.
     * @param slot The Smart Motion slot (0-3).
     * @return Maximum velocity as float.
     */
    float GetSmartMotionMaxVelocity(uint8_t slot);

    /**
     * @brief Get the maximum acceleration for Smart Motion in a given slot.
     * @param slot The Smart Motion slot (0-3).
     * @return Maximum acceleration as float.
     */
    float GetSmartMotionMaxAccel(uint8_t slot);

    /**
     * @brief Get the minimum velocity output for Smart Motion in a given slot.
     * @param slot The Smart Motion slot (0-3).
     * @return Minimum velocity output as float.
     */
    float GetSmartMotionMinVelOutput(uint8_t slot);

    /**
     * @brief Get the allowed closed-loop error for Smart Motion in a given slot.
     * @param slot The Smart Motion slot (0-3).
     * @return Allowed closed-loop error as float.
     */
    float GetSmartMotionAllowedClosedLoopError(uint8_t slot);

    /**
     * @brief Get the acceleration strategy for Smart Motion in a given slot.
     * @param slot The Smart Motion slot (0-3).
     * @return Acceleration strategy as float.
     */
    float GetSmartMotionAccelStrategy(uint8_t slot);

    /**
     * @brief Get the maximum accumulated I term for Smart Motion in a given slot.
     * @param slot The Smart Motion slot (0-3).
     * @return Maximum I term accumulation as float.
     */
    float GetIMaxAccum(uint8_t slot);

    /**
     * @brief Get the value for Slot 3 Placeholder 1 in a given slot.
     * @param slot The slot (0-3).
     * @return Placeholder value as float.
     */
    float GetSlot3Placeholder1(uint8_t slot);

    /**
     * @brief Get the value for Slot 3 Placeholder 2 in a given slot.
     * @param slot The slot (0-3).
     * @return Placeholder value as float.
     */
    float GetSlot3Placeholder2(uint8_t slot);

    /**
     * @brief Get the value for Slot 3 Placeholder 3 in a given slot.
     * @param slot The slot (0-3).
     * @return Placeholder value as float.
     */
    float GetSlot3Placeholder3(uint8_t slot);

    // Analog Sensor //

    /**
     * @brief Get the analog position conversion factor.
     * @return Analog position conversion factor as float.
     */
    float GetAnalogPositionConversion();

    /**
     * @brief Get the analog velocity conversion factor.
     * @return Analog velocity conversion factor as float.
     */
    float GetAnalogVelocityConversion();

    /**
     * @brief Get the analog average depth.
     * @return Analog average depth as uint16_t.
     */
    uint16_t GetAnalogAverageDepth();

    /**
     * @brief Get the analog sensor mode.
     * @return Analog sensor mode as uint8_t.
     */
    uint8_t GetAnalogSensorMode();

    /**
     * @brief Get the analog inversion state.
     * @return Analog inversion state as bool.
     */
    bool GetAnalogInverted();

    /**
     * @brief Get the analog sample delta.
     * @return Analog sample delta as uint16_t.
     */
    uint16_t GetAnalogSampleDelta();

    // Alternate Encoder //

    /**
     * @brief Get the data port configuration.
     * @return Data port config as uint8_t.
     */
    uint8_t GetDataPortConfig();

    /**
     * @brief Get the alternate encoder counts per revolution.
     * @return Alternate encoder counts per revolution as uint16_t.
     */
    uint16_t GetAltEncoderCountsPerRev();

    /**
     * @brief Get the alternate encoder average depth.
     * @return Alternate encoder average depth as uint8_t.
     */
    uint8_t GetAltEncoderAverageDepth();

    /**
     * @brief Get the alternate encoder sample delta.
     * @return Alternate encoder sample delta as uint8_t.
     */
    uint8_t GetAltEncoderSampleDelta();

    /**
     * @brief Get the alternate encoder inversion state.
     * @return Alternate encoder inversion state as bool.
     */
    bool GetAltEncoderInverted();

    /**
     * @brief Get the alternate encoder position factor.
     * @return Alternate encoder position factor as float.
     */
    float GetAltEncoderPositionFactor();

    /**
     * @brief Get the alternate encoder velocity factor.
     * @return Alternate encoder velocity factor as float.
     */
    float GetAltEncoderVelocityFactor();

    // Duty Cycle Absolute Encoder //

    /**
     * @brief Get the duty cycle position factor.
     * @return Duty cycle position factor as float.
     */
    float GetDutyCyclePositionFactor();

    /**
     * @brief Get the duty cycle velocity factor.
     * @return Duty cycle velocity factor as float.
     */
    float GetDutyCycleVelocityFactor();

    /**
     * @brief Get the duty cycle inversion state.
     * @return Duty cycle inversion state as bool.
     */
    bool GetDutyCycleInverted();

    /**
     * @brief Get the duty cycle average depth.
     * @return Duty cycle average depth as uint8_t.
     */
    uint8_t GetDutyCycleAverageDepth();

    /**
     * @brief Get the duty cycle prescalar.
     * @return Duty cycle prescalar as uint8_t.
     */
    uint8_t GetDutyCyclePrescalar();

    /**
     * @brief Get the duty cycle zero offset.
     * @return Duty cycle zero offset as float.
     */
    float GetDutyCycleZeroOffset();
};

#endif // SPARKBASE_HPP