/**
 * @file SparkBase.cpp
 * @brief Source file for the base class for controlling REV Robotics SPARK motor controllers
 * @author Grayson Arendt
 */

#include "SparkBase.hpp"

int SparkBase::soc = -1;

SparkBase::SparkBase(const std::string &interfaceName, uint8_t deviceId)
    : interfaceName(interfaceName), deviceId(deviceId)

{
    // Ensure deviceId is within valid range
    if (deviceId > 62)
    {
        throw std::out_of_range(RED "Invalid CAN bus ID. Must be between 0 and 62." RESET);
    }

    // Create CAN socket
    SparkBase::soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (SparkBase::soc < 0)
    {
        throw std::system_error(errno, std::generic_category(),
                                std::string(RED) + "Socket creation failed: " + strerror(errno) +
                                    "\nPossible causes:\n"
                                    "1. CAN modules not loaded\n"
                                    "2. System resource limitations" +
                                    std::string(RESET));
    }

    // Prepare CAN interface request structure
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interfaceName.c_str(), sizeof(ifr.ifr_name) - 1);

    // Get CAN interface index
    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        close(soc);
        throw std::runtime_error(std::string(RED) + "IOCTL failed: " + strerror(errno) +
                                 "\nPossible causes:\n1. CAN interface does not exist\n2. CAN bus not initialized\n3. CAN interface is not up" +
                                 std::string(RESET));
    }

    // Set up CAN address and bind socket to interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(soc);
        throw std::runtime_error(RED "Binding to interface failed: Another program may be using this interface." RESET);
    }
}

SparkBase::~SparkBase() { close(soc); }

void SparkBase::SendCanFrame(uint32_t deviceId, uint8_t dlc, const std::array<uint8_t, 8> &data) const
{
    // Prepare CAN frame structure
    struct can_frame frame = {};
    frame.can_id = deviceId | CAN_EFF_FLAG; // Use extended frame format
    frame.can_dlc = dlc;                    // Data length code
    std::memcpy(frame.data, data.data(), dlc);

    // Retry sending frame up to MAX_ATTEMPTS if buffer is full
    constexpr std::chrono::milliseconds WAIT_TIME(1);
    constexpr int MAX_ATTEMPTS = 1000;
    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt)
    {
        ssize_t bytesSent = write(soc, &frame, sizeof(frame));
        if (bytesSent == sizeof(frame))
        {
            return; // Frame successfully sent
        }
        if (bytesSent < 0)
        {
            // Retry if the buffer is full, otherwise throw error
            if (errno == ENOBUFS || errno == EAGAIN)
            {
                std::this_thread::sleep_for(WAIT_TIME);
                continue;
            }
            throw std::runtime_error(RED "Failed to send CAN frame: " + std::string(strerror(errno)) + RESET);
        }
    }
    throw std::runtime_error(RED "Failed to send CAN frame: Buffer consistently full." RESET);
}

void SparkBase::SendControlMessage(std::variant<MotorControl, SystemControl> command, std::string commandName, float value, std::optional<float> minValue, std::optional<float> maxValue) const
{
    // Validate the value
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(RED + commandName + " must be a finite number." + RESET);
    }
    if (minValue && maxValue && (value < *minValue || value > *maxValue))
    {
        throw std::out_of_range(RED + commandName + " must be between " + std::to_string(*minValue) + " and " + std::to_string(*maxValue) + RESET);
    }

    // Build arbitration ID and prepare CAN frame data
    uint32_t arbitrationId = 0;
    uint32_t valueBits;
    std::array<uint8_t, 8> data = {};

    // Set arbitration ID based on command type
    std::visit([&](auto &&cmd)
               {
                   using T = std::decay_t<decltype(cmd)>;
                   if constexpr (std::is_same_v<T, MotorControl> || std::is_same_v<T, SystemControl>)
                   {
                       arbitrationId = static_cast<uint32_t>(cmd) | deviceId;
                   } },
               command);

    // Copy value into CAN frame data
    std::memcpy(&valueBits, &value, sizeof(valueBits));
    std::memcpy(data.data(), &valueBits, sizeof(valueBits));

    // Send CAN frame with control message
    SendCanFrame(arbitrationId, 8, data);
}

uint64_t SparkBase::ReadPeriodicStatus(Status period) const
{
    constexpr int CACHE_TIMEOUT_MS = 100;  // Cache timeout in milliseconds
    constexpr int READ_TIMEOUT_US = 20000; // Timeout for reading CAN frame

    auto now = std::chrono::steady_clock::now();
    auto it = cachedStatus.find(period);

    // Return cached status if within timeout period
    if (it != cachedStatus.end())
    {
        auto &[value, timestamp] = it->second;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp).count() < CACHE_TIMEOUT_MS)
        {
            return value;
        }
    }

    // Wait for response and read CAN frame
    struct can_frame response;
    struct timeval tv = {0, READ_TIMEOUT_US};
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(soc, &read_fds);

    int ret = select(soc + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0)
    {
        ssize_t bytesRead = read(soc, &response, sizeof(response));
        if (bytesRead > 0)
        {
            uint32_t receivedArbitrationId = response.can_id & CAN_EFF_MASK;
            uint32_t expectedArbitrationId = static_cast<uint32_t>(period) | deviceId;

            // If the arbitration ID matches, update the cached value
            if (receivedArbitrationId == expectedArbitrationId)
            {
                uint64_t newValue = 0;
                for (int i = 0; i < 8; i++)
                {
                    newValue |= static_cast<uint64_t>(response.data[i]) << (8 * i);
                }
                cachedStatus[period] = {newValue, now};
                return newValue;
            }
        }
    }

    // Return cached value if no new data received
    if (it != cachedStatus.end())
    {
        return it->second.first;
    }

    return 0; // Default return if no cached or new value
}

void SparkBase::SetParameter(
    Parameter parameterId,
    uint8_t parameterType,
    std::string parameterName,
    std::variant<float, uint32_t, uint16_t, uint8_t, bool> value,
    std::optional<float> minValue,
    std::optional<float> maxValue,
    std::optional<std::string> customErrorMessage)
{
    // Generate arbitration ID based on parameter ID and deviceId
    uint32_t arbitrationId = 0x205C000 | (static_cast<uint32_t>(parameterId) << 6) | deviceId;
    std::array<uint8_t, 8> data = {};

    // Lambda to handle range validation errors
    auto throwRangeError = [&](auto min, auto max)
    {
        if (customErrorMessage)
            throw std::out_of_range(RED + *customErrorMessage + RESET);
        else
            throw std::out_of_range(RED + parameterName + " must be between " + std::to_string(min) + " and " + std::to_string(max) + RESET);
    };

    // Process the value based on its type and fill CAN data
    std::visit([&](auto &&v)
               {
                   using T = std::decay_t<decltype(v)>;
                   if constexpr (std::is_same_v<T, float>)
                   {
                       if (!std::isfinite(v)) // Ensure float is valid
                           throw std::invalid_argument(RED + parameterName + " must be a finite number." + RESET);
                       if (minValue && maxValue && (v < minValue.value() || v > maxValue.value()))
                           throwRangeError(minValue.value(), maxValue.value());
                       std::memcpy(data.data(), &v, sizeof(v)); // Copy float to CAN data
                   }
                   else if constexpr (std::is_integral_v<T>)
                   {
                       std::memcpy(data.data(), &v, sizeof(v)); // Copy integer to CAN data
                   }
                   else if constexpr (std::is_same_v<T, bool>)
                   {
                       data[0] = v ? 1 : 0; // Handle boolean type
                   }
                   else
                   {
                       throw std::invalid_argument(RED "Unsupported value type." RESET);
                   } },
               value);

    data[4] = parameterType;              // Add parameter type to CAN data
    SendCanFrame(arbitrationId, 5, data); // Send CAN frame with parameter data
}

std::variant<float, uint32_t, bool> SparkBase::ReadParameter(Parameter parameterId)
{
    constexpr int READ_TIMEOUT_US = 20000; // Timeout for reading CAN frame

    // Prepare and send request for parameter value
    struct can_frame request;
    std::memset(&request, 0, sizeof(request)); // Zero out request frame
    uint32_t requestArbitrationId = 0x205C000 | (static_cast<uint32_t>(parameterId) << 6) | deviceId;
    request.can_id = requestArbitrationId | CAN_EFF_FLAG; // Use extended frame format
    request.can_dlc = 0;                                  // Set data length to 0 (empty request)

    // Send empty CAN frame
    if (write(soc, &request, sizeof(request)) < 0)
    {
        perror("Error sending CAN message");
        return uint32_t(0); // Return default on error
    }

    // Wait for response
    struct can_frame response;
    struct timeval tv = {0, READ_TIMEOUT_US};
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(soc, &read_fds);

    int ret = select(soc + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret > 0)
    {
        ssize_t bytesRead = read(soc, &response, sizeof(response));
        if (bytesRead > 0)
        {
            uint32_t receivedArbitrationId = response.can_id & CAN_EFF_MASK;
            if (receivedArbitrationId == requestArbitrationId)
            {
                // Parse response data based on type in response.data[4]
                uint8_t dataType = response.data[4];
                std::variant<float, uint32_t, bool> newValue;
                switch (dataType)
                {
                case 0x01:
                {
                    uint32_t uintValue = 0;
                    for (int i = 0; i < 4; ++i)
                    {
                        uintValue |= static_cast<uint32_t>(response.data[i]) << (8 * i);
                    }
                    newValue = uintValue;
                    break;
                }
                case 0x02:
                {
                    float floatValue;
                    std::memcpy(&floatValue, response.data, sizeof(float));
                    newValue = floatValue;
                    break;
                }
                case 0x03:
                {
                    bool boolValue = response.data[0] != 0;
                    newValue = boolValue;
                    break;
                }
                default:
                    return uint32_t(0); // Return default for unknown types
                }
                return newValue;
            }
        }
    }

    return uint32_t(0); // Return default if no response
}

void SparkBase::Heartbeat()
{
    struct can_frame frame = {};
    frame.can_id = 0x2052C80 | CAN_EFF_FLAG;
    frame.can_dlc = 8;

    // Fill data with 0xFF
    std::memset(frame.data, 0xFF, sizeof(frame.data));

    // Retry sending frame up to MAX_ATTEMPTS if buffer is full
    constexpr std::chrono::milliseconds WAIT_TIME(1);
    constexpr int MAX_ATTEMPTS = 1000;
    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt)
    {
        // Send the frame
        ssize_t bytesSent = write(SparkBase::soc, &frame, sizeof(frame));
        if (bytesSent == sizeof(frame))
        {
            return; // Frame successfully sent
        }

        if (bytesSent < 0)
        {
            // Retry if the buffer is full, otherwise throw error
            if (errno == ENOBUFS || errno == EAGAIN)
            {
                std::this_thread::sleep_for(WAIT_TIME);
                continue;
            }
            throw std::runtime_error(RED "Failed to send CAN frame: " + std::string(strerror(errno)) + RESET);
        }
    }
    throw std::runtime_error(RED "Failed to send CAN frame: Buffer consistently full." RESET);
}

void SparkBase::BurnFlash()
{
    uint32_t arbitrationId = static_cast<uint32_t>(SystemControl::BurnFlash) + deviceId;

    std::array<uint8_t, 8> data = {0xA3, 0x3A};

    SendCanFrame(arbitrationId, 2, data);
}

void SparkBase::FactoryDefaults()
{
    uint32_t arbitrationId =
        static_cast<uint32_t>(SystemControl::FactoryDefaults) + deviceId;

    std::array<uint8_t, 8> data = {0x01};

    SendCanFrame(arbitrationId, 5, data);
}

void SparkBase::FactoryReset()
{
    uint32_t arbitrationId = static_cast<uint32_t>(SystemControl::FactoryReset) + deviceId;

    std::array<uint8_t, 8> data = {0x01};

    SendCanFrame(arbitrationId, 5, data);
}

void SparkBase::Identify()
{
    uint32_t arbitrationId = static_cast<uint32_t>(SystemControl::Identify) + deviceId;

    std::array<uint8_t, 8> data = {};

    SendCanFrame(arbitrationId, 4, data);
}

void SparkBase::ResetFaults()
{
    SendControlMessage(SystemControl::ResetFaults, "ResetFaults", 0.0f);
}

void SparkBase::ClearStickyFaults()
{
    SendControlMessage(SystemControl::ClearStickyFaults, "ClearStickyFaults", 0.0f);
}

// MotorControl //

void SparkBase::SetSetpoint(float setpoint)
{
    SendControlMessage(MotorControl::Setpoint, "Setpoint", setpoint);
}

void SparkBase::SetDutyCycle(float dutyCycle)
{
    SendControlMessage(MotorControl::DutyCycle, "Duty Cycle", dutyCycle, -1.0f, 1.0f);
}

void SparkBase::SetVelocity(float velocity)
{
    SendControlMessage(MotorControl::Velocity, "Velocity", velocity);
}

void SparkBase::SetSmartVelocity(float smartVelocity)
{
    SendControlMessage(MotorControl::SmartVelocity, "Smart Velocity", smartVelocity);
}

void SparkBase::SetPosition(float position)
{
    SendControlMessage(MotorControl::Position, "Position", position);
}

void SparkBase::SetVoltage(float voltage)
{
    SendControlMessage(MotorControl::Voltage, "Voltage", voltage);
}

void SparkBase::SetCurrent(float current)
{
    SendControlMessage(MotorControl::Current, "Current", current);
}

void SparkBase::SetSmartMotion(float smartMotion)
{
    SendControlMessage(MotorControl::SmartMotion, "Smart Motion", smartMotion);
}

// Status //

// Period 0
float SparkBase::GetDutyCycle() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period0);
    int16_t DutyCycle = static_cast<int16_t>(status & 0xFFFF);

    return static_cast<float>(DutyCycle) / 32768.0f;
}

uint16_t SparkBase::GetFaults() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period0);
    return (status >> 16) & 0xFFFF;
}

uint16_t SparkBase::GetStickyFaults() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period0);
    return (status >> 32) & 0xFFFF;
}

bool SparkBase::GetInverted() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period0);
    return (rawData >> 56) & 0x01;
}

bool SparkBase::GetIdleMode() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period0);
    return (rawData >> 57) & 0x01;
}

bool SparkBase::IsFollower() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period0);
    return (rawData >> 58) & 0x01;
}

// Period 1 //
float SparkBase::GetVelocity() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    return *reinterpret_cast<const float *>(&status);
}

float SparkBase::GetTemperature() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    return static_cast<float>((status >> 32) & 0xFF);
}

float SparkBase::GetVoltage() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    uint16_t voltage = (status >> 40) & 0xFFF;

    return static_cast<float>(voltage) / 128.0f;
}

float SparkBase::GetCurrent() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period1);
    uint16_t current = (status >> 52) & 0xFFF;

    return static_cast<float>(current) / 32.0f;
}

// Period 2 //
float SparkBase::GetPosition() const
{
    uint64_t rawData = ReadPeriodicStatus(Status::Period2);

    return *reinterpret_cast<const float *>(&rawData);
}

// Period 3 //
float SparkBase::GetAnalogVoltage() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    uint16_t analogVoltage = status & 0x3FF;

    return static_cast<float>(analogVoltage) / 1024.0f;
}

float SparkBase::GetAnalogVelocity() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    uint32_t analogVelocity = (status >> 10) & 0x3FFFFF;

    return static_cast<float>(analogVelocity) / 1.0f;
}

float SparkBase::GetAnalogPosition() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period3);
    return *reinterpret_cast<const float *>(&status) / 128.0f;
}

// Period 4 //
float SparkBase::GetAlternateEncoderVelocity() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period4);
    return *reinterpret_cast<const float *>(&status);
}

float SparkBase::GetAlternateEncoderPosition() const
{
    uint64_t status = ReadPeriodicStatus(Status::Period4);
    return *reinterpret_cast<const float *>((const char *)&status + 4);
}

// Parameters //

// Basic //

void SparkBase::SetMotorType(MotorType type)
{
    SetParameter(Parameter::kMotorType, PARAM_TYPE_UINT, "Motor Type", static_cast<uint8_t>(type), 0, 1,
                 "Invalid motor type. Must be 0 (Brushed) or 1 (Brushless).");
}

void SparkBase::SetSensorType(SensorType type)
{
    SetParameter(Parameter::kSensorType, PARAM_TYPE_UINT, "Sensor Type", static_cast<uint8_t>(type), 0, 2,
                 "Invalid sensor type. Must be 0 (No Sensor), 1 (Hall Sensor), or 2 (Encoder).");
}

void SparkBase::SetIdleMode(IdleMode mode)
{
    SetParameter(Parameter::kIdleMode, PARAM_TYPE_UINT, "Idle Mode", static_cast<uint8_t>(mode), 0, 1,
                 "Invalid idle mode. Must be 0 (Coast) or 1 (Brake).");
}

void SparkBase::SetInputDeadband(float deadband)
{
    SetParameter(Parameter::kInputDeadband, PARAM_TYPE_FLOAT, "Input Deadband", deadband);
}

void SparkBase::SetInverted(bool inverted)
{
    SetParameter(Parameter::kInverted, PARAM_TYPE_BOOL, "Inverted", inverted);
}

void SparkBase::SetRampRate(float rate)
{
    SetParameter(Parameter::kRampRate, PARAM_TYPE_FLOAT, "Ramp Rate", rate);
}

// Advanced //

void SparkBase::SetMotorKv(uint16_t kv)
{
    SetParameter(Parameter::kMotorKv, PARAM_TYPE_UINT, "Motor Kv", kv);
}

void SparkBase::SetMotorR(uint16_t r)
{
    SetParameter(Parameter::kMotorR, PARAM_TYPE_UINT, "Motor Resistance", r);
}

void SparkBase::SetMotorL(uint16_t l)
{
    SetParameter(Parameter::kMotorL, PARAM_TYPE_UINT, "Motor Inductance", l);
}

// Closed Loop //

void SparkBase::SetCtrlType(CtrlType type)
{
    SetParameter(Parameter::kCtrlType, PARAM_TYPE_UINT, "Control Type", static_cast<uint8_t>(type), 0, 3,
                 "Invalid control type. Must be 0 (Duty Cycle), 1 (Velocity), 2 (Voltage), or 3 (Position).");
}

void SparkBase::SetFeedbackSensorPID0(uint16_t sensor)
{
    SetParameter(Parameter::kFeedbackSensorPID0, PARAM_TYPE_UINT, "Feedback Sensor PID0", sensor);
}

void SparkBase::SetClosedLoopVoltageMode(uint8_t mode)
{
    SetParameter(Parameter::kClosedLoopVoltageMode, PARAM_TYPE_UINT, "Closed Loop Voltage Mode", mode, 0, 2,
                 "Invalid closed loop voltage mode. Must be 0 (Disabled), 1 (Control Loop Voltage Output Mode) or 2 (Voltage Compensation Mode).");
}

void SparkBase::SetCompensatedNominalVoltage(float voltage)
{
    SetParameter(Parameter::kCompensatedNominalVoltage, PARAM_TYPE_FLOAT, "Compensated Nominal Voltage", voltage);
}

void SparkBase::SetPositionPIDWrapEnable(bool enable)
{
    SetParameter(Parameter::kPositionPIDWrapEnable, PARAM_TYPE_BOOL, "Position PID Wrap Enable", enable);
}

void SparkBase::SetPositionPIDMinInput(float minInput)
{
    SetParameter(Parameter::kPositionPIDMinInput, PARAM_TYPE_FLOAT, "Position PID Min Input", minInput);
}

void SparkBase::SetPositionPIDMaxInput(float maxInput)
{
    SetParameter(Parameter::kPositionPIDMaxInput, PARAM_TYPE_FLOAT, "Position PID Max Input", maxInput);
}

// Brushless //

void SparkBase::SetPolePairs(uint16_t pairs)
{
    SetParameter(Parameter::kPolePairs, PARAM_TYPE_UINT, "Pole Pairs", pairs);
}

// Current Limit //

void SparkBase::SetCurrentChop(float chop)
{
    SetParameter(Parameter::kCurrentChop, PARAM_TYPE_FLOAT, "Current Chop", chop, 0, 125);
}

void SparkBase::SetCurrentChopCycles(uint16_t cycles)
{
    SetParameter(Parameter::kCurrentChopCycles, PARAM_TYPE_UINT, "Current Chop Cycles", cycles);
}

void SparkBase::SetSmartCurrentStallLimit(uint16_t limit)
{
    SetParameter(Parameter::kSmartCurrentStallLimit, PARAM_TYPE_UINT, "Smart Current Stall Limit", limit);
}

void SparkBase::SetSmartCurrentFreeLimit(uint16_t limit)
{
    SetParameter(Parameter::kSmartCurrentFreeLimit, PARAM_TYPE_UINT, "Smart Current Free Limit", limit);
}

void SparkBase::SetSmartCurrentConfig(uint16_t config)
{
    SetParameter(Parameter::kSmartCurrentConfig, PARAM_TYPE_UINT, "Smart Current Config", config);
}

// PIDF //

void SparkBase::SetP(uint8_t slot, float p)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kP_0,
        Parameter::kP_1,
        Parameter::kP_2,
        Parameter::kP_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "P", p);
}

void SparkBase::SetI(uint8_t slot, float i)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kI_0,
        Parameter::kI_1,
        Parameter::kI_2,
        Parameter::kI_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "I", i);
}

void SparkBase::SetD(uint8_t slot, float d)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kD_0,
        Parameter::kD_1,
        Parameter::kD_2,
        Parameter::kD_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "D", d);
}

void SparkBase::SetF(uint8_t slot, float f)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kF_0,
        Parameter::kF_1,
        Parameter::kF_2,
        Parameter::kF_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "F", f);
}

void SparkBase::SetIZone(uint8_t slot, float iZone)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kIZone_0,
        Parameter::kIZone_1,
        Parameter::kIZone_2,
        Parameter::kIZone_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "IZone", iZone);
}

void SparkBase::SetDFilter(uint8_t slot, float dFilter)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kDFilter_0,
        Parameter::kDFilter_1,
        Parameter::kDFilter_2,
        Parameter::kDFilter_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "DFilter", dFilter);
}

void SparkBase::SetOutputMin(uint8_t slot, float min)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kOutputMin_0,
        Parameter::kOutputMin_1,
        Parameter::kOutputMin_2,
        Parameter::kOutputMin_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Output Min", min);
}

void SparkBase::SetOutputMax(uint8_t slot, float max)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kOutputMax_0,
        Parameter::kOutputMax_1,
        Parameter::kOutputMax_2,
        Parameter::kOutputMax_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Output Max", max);
}

// Limits //

void SparkBase::SetHardLimitFwdEn(bool enable)
{
    SetParameter(Parameter::kHardLimitFwdEn, PARAM_TYPE_BOOL, "Hard Limit Forward Enable", enable);
}

void SparkBase::SetHardLimitRevEn(bool enable)
{
    SetParameter(Parameter::kHardLimitRevEn, PARAM_TYPE_BOOL, "Hard Limit Reverse Enable", enable);
}

void SparkBase::SetLimitSwitchFwdPolarity(bool polarity)
{
    SetParameter(Parameter::kLimitSwitchFwdPolarity, PARAM_TYPE_BOOL, "Limit Switch Forward Polarity", polarity);
}

void SparkBase::SetLimitSwitchRevPolarity(bool polarity)
{
    SetParameter(Parameter::kLimitSwitchRevPolarity, PARAM_TYPE_BOOL, "Limit Switch Reverse Polarity", polarity);
}

void SparkBase::SetSoftLimitFwdEn(bool enable)
{
    SetParameter(Parameter::kSoftLimitFwdEn, PARAM_TYPE_BOOL, "Soft Limit Forward Enable", enable);
}

void SparkBase::SetSoftLimitRevEn(bool enable)
{
    SetParameter(Parameter::kSoftLimitRevEn, PARAM_TYPE_BOOL, "Soft Limit Reverse Enable", enable);
}

void SparkBase::SetSoftLimitFwd(float limit)
{
    SetParameter(Parameter::kSoftLimitFwd, PARAM_TYPE_FLOAT, "Soft Limit Forward", limit);
}

void SparkBase::SetSoftLimitRev(float limit)
{
    SetParameter(Parameter::kSoftLimitRev, PARAM_TYPE_FLOAT, "Soft Limit Reverse", limit);
}

// Follower //

void SparkBase::SetFollowerID(uint32_t id)
{
    SetParameter(Parameter::kFollowerID, PARAM_TYPE_UINT, "Follower ID", id);
}

void SparkBase::SetFollowerConfig(uint32_t config)
{
    SetParameter(Parameter::kFollowerConfig, PARAM_TYPE_UINT, "Follower Config", config);
}

// Encoder Port //

void SparkBase::SetEncoderCountsPerRev(uint16_t counts)
{
    SetParameter(Parameter::kEncoderCountsPerRev, PARAM_TYPE_UINT, "Encoder Counts Per Revolution", counts);
}

void SparkBase::SetEncoderAverageDepth(uint8_t depth)
{
    SetParameter(Parameter::kEncoderAverageDepth, PARAM_TYPE_UINT, "Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetEncoderSampleDelta(uint8_t delta)
{
    SetParameter(Parameter::kEncoderSampleDelta, PARAM_TYPE_UINT, "Encoder Sample Delta", delta, 1, 255);
}

void SparkBase::SetEncoderInverted(bool inverted)
{
    SetParameter(Parameter::kEncoderInverted, PARAM_TYPE_BOOL, "Encoder Inverted", inverted);
}

void SparkBase::SetPositionConversionFactor(float factor)
{
    SetParameter(Parameter::kPositionConversionFactor, PARAM_TYPE_FLOAT, "Position Conversion Factor", factor);
}

void SparkBase::SetVelocityConversionFactor(float factor)
{
    SetParameter(Parameter::kVelocityConversionFactor, PARAM_TYPE_FLOAT, "Velocity Conversion Factor", factor);
}

void SparkBase::SetClosedLoopRampRate(float rampRate)
{
    SetParameter(Parameter::kClosedLoopRampRate, PARAM_TYPE_FLOAT, "Closed Loop Ramp Rate", rampRate);
}

void SparkBase::SetHallSensorSampleRate(float rate)
{
    SetParameter(Parameter::kHallSensorSampleRate, PARAM_TYPE_FLOAT, "Hall Sensor Sample Rate", rate);
}

void SparkBase::SetHallSensorAverageDepth(uint16_t depth)
{
    SetParameter(Parameter::kHallSensorAverageDepth, PARAM_TYPE_UINT, "Hall Sensor Average Depth", depth);
}

// Smart Motion //

void SparkBase::SetSmartMotionMaxVelocity(uint8_t slot, float maxVel)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSmartMotionMaxVelocity_0,
        Parameter::kSmartMotionMaxVelocity_1,
        Parameter::kSmartMotionMaxVelocity_2,
        Parameter::kSmartMotionMaxVelocity_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Max Velocity", maxVel);
}

void SparkBase::SetSmartMotionMaxAccel(uint8_t slot, float maxAccel)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSmartMotionMaxAccel_0,
        Parameter::kSmartMotionMaxAccel_1,
        Parameter::kSmartMotionMaxAccel_2,
        Parameter::kSmartMotionMaxAccel_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Max Accel", maxAccel);
}

void SparkBase::SetSmartMotionMinVelOutput(uint8_t slot, float minVel)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSmartMotionMinVelOutput_0,
        Parameter::kSmartMotionMinVelOutput_1,
        Parameter::kSmartMotionMinVelOutput_2,
        Parameter::kSmartMotionMinVelOutput_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Min Vel Output", minVel);
}

void SparkBase::SetSmartMotionAllowedClosedLoopError(uint8_t slot, float error)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSmartMotionAllowedClosedLoopError_0,
        Parameter::kSmartMotionAllowedClosedLoopError_1,
        Parameter::kSmartMotionAllowedClosedLoopError_2,
        Parameter::kSmartMotionAllowedClosedLoopError_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Allowed Close Loop Error", error);
}

void SparkBase::SetSmartMotionAccelStrategy(uint8_t slot, float strategy)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSmartMotionAccelStrategy_0,
        Parameter::kSmartMotionAccelStrategy_1,
        Parameter::kSmartMotionAccelStrategy_2,
        Parameter::kSmartMotionAccelStrategy_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Smart Motion Accel Strategy", strategy);
}

void SparkBase::SetIMaxAccum(uint8_t slot, float maxAccum)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kIMaxAccum_0,
        Parameter::kIMaxAccum_1,
        Parameter::kIMaxAccum_2,
        Parameter::kIMaxAccum_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "IMaxAccum", maxAccum);
}

void SparkBase::SetSlot3Placeholder1(uint8_t slot, float value)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSlot3Placeholder1_0,
        Parameter::kSlot3Placeholder1_1,
        Parameter::kSlot3Placeholder1_2,
        Parameter::kSlot3Placeholder1_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 1", value);
}

void SparkBase::SetSlot3Placeholder2(uint8_t slot, float value)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSlot3Placeholder2_0,
        Parameter::kSlot3Placeholder2_1,
        Parameter::kSlot3Placeholder2_2,
        Parameter::kSlot3Placeholder2_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 2", value);
}

void SparkBase::SetSlot3Placeholder3(uint8_t slot, float value)
{
    static const std::array<Parameter, 4> params = {
        Parameter::kSlot3Placeholder3_0,
        Parameter::kSlot3Placeholder3_1,
        Parameter::kSlot3Placeholder3_2,
        Parameter::kSlot3Placeholder3_3};

    if (slot >= params.size())
    {
        throw std::out_of_range(RED "Invalid slot number. Max value is 3." RESET);
    }

    SetParameter(params[slot], PARAM_TYPE_FLOAT, "Slot 3 Placeholder 3", value);
}

// Analog Sensor //

void SparkBase::SetAnalogPositionConversion(float factor)
{
    SetParameter(Parameter::kAnalogPositionConversion, PARAM_TYPE_FLOAT, "Analog Position Conversion", factor);
}

void SparkBase::SetAnalogVelocityConversion(float factor)
{
    SetParameter(Parameter::kAnalogVelocityConversion, PARAM_TYPE_FLOAT, "Analog Velocity Conversion", factor);
}

void SparkBase::SetAnalogAverageDepth(uint16_t depth)
{
    SetParameter(Parameter::kAnalogAverageDepth, PARAM_TYPE_UINT, "Analog Average Depth", depth);
}

void SparkBase::SetAnalogSensorMode(uint8_t mode)
{
    SetParameter(Parameter::kAnalogSensorMode, PARAM_TYPE_UINT, "Analog Sensor Mode", mode, 0, 1,
                 "Invalid analog sensor mode. Must be 0 (Absolute) or 1 (Relative).");
}

void SparkBase::SetAnalogInverted(bool inverted)
{
    SetParameter(Parameter::kAnalogInverted, PARAM_TYPE_BOOL, "Analog Inverted", inverted);
}

void SparkBase::SetAnalogSampleDelta(uint16_t delta)
{
    SetParameter(Parameter::kAnalogSampleDelta, PARAM_TYPE_UINT, "Analog Sample Delta", delta);
}

// Alternate Encoder //

void SparkBase::SetDataPortConfig(uint8_t config)
{
    SetParameter(Parameter::kDataPortConfig, PARAM_TYPE_UINT, "Data Port Config", config, 0, 1,
                 "Invalid data port config. Must be 0 (Default) or 1 (Alternate Encoder Mode).");
}

void SparkBase::SetAltEncoderCountsPerRev(uint16_t counts)
{
    SetParameter(Parameter::kAltEncoderCountsPerRev, PARAM_TYPE_UINT, "Alternate Encoder Counts Per Revolution", counts);
}

void SparkBase::SetAltEncoderAverageDepth(uint8_t depth)
{
    SetParameter(Parameter::kAltEncoderAverageDepth, PARAM_TYPE_UINT, "Alternate Encoder Average Depth", depth, 1, 64);
}

void SparkBase::SetAltEncoderSampleDelta(uint8_t delta)
{
    SetParameter(Parameter::kAltEncoderSampleDelta, PARAM_TYPE_UINT, "Alternate Encoder Sample Delta", delta, 1, 255);
}

void SparkBase::SetAltEncoderInverted(bool inverted)
{
    SetParameter(Parameter::kAltEncoderInverted, PARAM_TYPE_BOOL, "Alternate Encoder Inverted", inverted);
}

void SparkBase::SetAltEncoderPositionFactor(float factor)
{
    SetParameter(Parameter::kAltEncoderPositionFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Position Factor", factor);
}

void SparkBase::SetAltEncoderVelocityFactor(float factor)
{
    SetParameter(Parameter::kAltEncoderVelocityFactor, PARAM_TYPE_FLOAT, "Alternate Encoder Velocity Factor", factor);
}

// Duty Cycle Absolute Encoder //

void SparkBase::SetDutyCyclePositionFactor(float factor)
{
    SetParameter(Parameter::kDutyCyclePositionFactor, PARAM_TYPE_FLOAT, "Duty Cycle Position Factor", factor);
}

void SparkBase::SetDutyCycleVelocityFactor(float factor)
{
    SetParameter(Parameter::kDutyCycleVelocityFactor, PARAM_TYPE_FLOAT, "Duty Cycle Velocity Factor", factor);
}

void SparkBase::SetDutyCycleInverted(bool inverted)
{
    SetParameter(Parameter::kDutyCycleInverted, PARAM_TYPE_BOOL, "Duty Cycle Inverted", inverted);
}

void SparkBase::SetDutyCycleAverageDepth(uint8_t depth)
{
    SetParameter(Parameter::kDutyCycleAverageDepth, PARAM_TYPE_UINT, "Duty Cycle Average Depth", depth, 0, 7,
                 "Invalid average depth. Must be 0 (1 bit), 2 (2 bits), 3 (4 bits), 4 (8 bits), 5 (16 bits), 6 (32 bits), or 7 (64 bits).");
}

void SparkBase::SetDutyCyclePrescalar(uint8_t prescalar)
{
    SetParameter(Parameter::kDutyCyclePrescalar, PARAM_TYPE_UINT, "Duty Cycle Prescalar", prescalar, 0, 71);
}

void SparkBase::SetDutyCycleZeroOffset(float offset)
{
    SetParameter(Parameter::kDutyCycleZeroOffset, PARAM_TYPE_FLOAT, "Duty Cycle Zero Offset", offset, 0.0f, 1.0f);
}

// Parameter Getters //

uint8_t SparkBase::GetMotorType()
{
    auto result = ReadParameter(Parameter::kMotorType);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for MotorType");
}

uint8_t SparkBase::GetSensorType()
{
    auto result = ReadParameter(Parameter::kSensorType);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for SensorType");
}

uint8_t SparkBase::GetIdleMode()
{
    auto result = ReadParameter(Parameter::kIdleMode);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for IdleMode");
}

float SparkBase::GetInputDeadband()
{
    auto result = ReadParameter(Parameter::kInputDeadband);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for InputDeadband");
}

bool SparkBase::GetInverted()
{
    auto result = ReadParameter(Parameter::kInverted);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for Inverted");
}

float SparkBase::GetRampRate()
{
    auto result = ReadParameter(Parameter::kRampRate);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for RampRate");
}

// Advanced //

uint16_t SparkBase::GetMotorKv()
{
    auto result = ReadParameter(Parameter::kMotorKv);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for MotorKv");
}

uint16_t SparkBase::GetMotorR()
{
    auto result = ReadParameter(Parameter::kMotorR);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for MotorR");
}

uint16_t SparkBase::GetMotorL()
{
    auto result = ReadParameter(Parameter::kMotorL);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for MotorL");
}

// Closed Loop //

uint8_t SparkBase::GetCtrlType()
{
    auto result = ReadParameter(Parameter::kCtrlType);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for CtrlType");
}

uint16_t SparkBase::GetFeedbackSensorPID0()
{
    auto result = ReadParameter(Parameter::kFeedbackSensorPID0);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for FeedbackSensorPID0");
}

uint8_t SparkBase::GetClosedLoopVoltageMode()
{
    auto result = ReadParameter(Parameter::kClosedLoopVoltageMode);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for ClosedLoopVoltageMode");
}

float SparkBase::GetCompensatedNominalVoltage()
{
    auto result = ReadParameter(Parameter::kCompensatedNominalVoltage);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for CompensatedNominalVoltage");
}

bool SparkBase::GetPositionPIDWrapEnable()
{
    auto result = ReadParameter(Parameter::kPositionPIDWrapEnable);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for PositionPIDWrapEnable");
}

float SparkBase::GetPositionPIDMinInput()
{
    auto result = ReadParameter(Parameter::kPositionPIDMinInput);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for PositionPIDMinInput");
}

float SparkBase::GetPositionPIDMaxInput()
{
    auto result = ReadParameter(Parameter::kPositionPIDMaxInput);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for PositionPIDMaxInput");
}

// Brushless //

uint16_t SparkBase::GetPolePairs()
{
    auto result = ReadParameter(Parameter::kPolePairs);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for PolePairs");
}

// Current Limit //

float SparkBase::GetCurrentChop()
{
    auto result = ReadParameter(Parameter::kCurrentChop);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for CurrentChop");
}

uint16_t SparkBase::GetCurrentChopCycles()
{
    auto result = ReadParameter(Parameter::kCurrentChopCycles);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for CurrentChopCycles");
}

uint16_t SparkBase::GetSmartCurrentStallLimit()
{
    auto result = ReadParameter(Parameter::kSmartCurrentStallLimit);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for SmartCurrentStallLimit");
}

uint16_t SparkBase::GetSmartCurrentFreeLimit()
{
    auto result = ReadParameter(Parameter::kSmartCurrentFreeLimit);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for SmartCurrentFreeLimit");
}

uint16_t SparkBase::GetSmartCurrentConfig()
{
    auto result = ReadParameter(Parameter::kSmartCurrentConfig);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for SmartCurrentConfig");
}

float SparkBase::GetP(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kP_0, Parameter::kP_1, Parameter::kP_2, Parameter::kP_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for P");
}

float SparkBase::GetI(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kI_0, Parameter::kI_1, Parameter::kI_2, Parameter::kI_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for I");
}

float SparkBase::GetD(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kD_0, Parameter::kD_1, Parameter::kD_2, Parameter::kD_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for D");
}

float SparkBase::GetF(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kF_0, Parameter::kF_1, Parameter::kF_2, Parameter::kF_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for F");
}

float SparkBase::GetIZone(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kIZone_0, Parameter::kIZone_1, Parameter::kIZone_2, Parameter::kIZone_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for IZone");
}

float SparkBase::GetDFilter(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kDFilter_0, Parameter::kDFilter_1, Parameter::kDFilter_2, Parameter::kDFilter_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for DFilter");
}

float SparkBase::GetOutputMin(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kOutputMin_0, Parameter::kOutputMin_1, Parameter::kOutputMin_2, Parameter::kOutputMin_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for OutputMin");
}

float SparkBase::GetOutputMax(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kOutputMax_0, Parameter::kOutputMax_1, Parameter::kOutputMax_2, Parameter::kOutputMax_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for OutputMax");
}

bool SparkBase::GetHardLimitFwdEn()
{
    auto result = ReadParameter(Parameter::kHardLimitFwdEn);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for HardLimitFwdEn");
}

bool SparkBase::GetHardLimitRevEn()
{
    auto result = ReadParameter(Parameter::kHardLimitRevEn);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for HardLimitRevEn");
}

bool SparkBase::GetLimitSwitchFwdPolarity()
{
    auto result = ReadParameter(Parameter::kLimitSwitchFwdPolarity);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for LimitSwitchFwdPolarity");
}

bool SparkBase::GetLimitSwitchRevPolarity()
{
    auto result = ReadParameter(Parameter::kLimitSwitchRevPolarity);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for LimitSwitchRevPolarity");
}

bool SparkBase::GetSoftLimitFwdEn()
{
    auto result = ReadParameter(Parameter::kSoftLimitFwdEn);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for SoftLimitFwdEn");
}

bool SparkBase::GetSoftLimitRevEn()
{
    auto result = ReadParameter(Parameter::kSoftLimitRevEn);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for SoftLimitRevEn");
}

float SparkBase::GetSoftLimitFwd()
{
    auto result = ReadParameter(Parameter::kSoftLimitFwd);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SoftLimitFwd");
}

float SparkBase::GetSoftLimitRev()
{
    auto result = ReadParameter(Parameter::kSoftLimitRev);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SoftLimitRev");
}

uint32_t SparkBase::GetFollowerID()
{
    auto result = ReadParameter(Parameter::kFollowerID);
    if (std::holds_alternative<uint32_t>(result))
    {
        return std::get<uint32_t>(result);
    }
    throw std::runtime_error("Invalid type returned for FollowerID");
}

uint32_t SparkBase::GetFollowerConfig()
{
    auto result = ReadParameter(Parameter::kFollowerConfig);
    if (std::holds_alternative<uint32_t>(result))
    {
        return std::get<uint32_t>(result);
    }
    throw std::runtime_error("Invalid type returned for FollowerConfig");
}

float SparkBase::GetClosedLoopRampRate()
{
    auto result = ReadParameter(Parameter::kClosedLoopRampRate);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for ClosedLoopRampRate");
}

uint16_t SparkBase::GetEncoderCountsPerRev()
{
    auto result = ReadParameter(Parameter::kEncoderCountsPerRev);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for EncoderCountsPerRev");
}

uint8_t SparkBase::GetEncoderAverageDepth()
{
    auto result = ReadParameter(Parameter::kEncoderAverageDepth);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for EncoderAverageDepth");
}

uint8_t SparkBase::GetEncoderSampleDelta()
{
    auto result = ReadParameter(Parameter::kEncoderSampleDelta);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for EncoderSampleDelta");
}

bool SparkBase::GetEncoderInverted()
{
    auto result = ReadParameter(Parameter::kEncoderInverted);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for EncoderInverted");
}

float SparkBase::GetPositionConversionFactor()
{
    auto result = ReadParameter(Parameter::kPositionConversionFactor);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for PositionConversionFactor");
}

float SparkBase::GetVelocityConversionFactor()
{
    auto result = ReadParameter(Parameter::kVelocityConversionFactor);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for VelocityConversionFactor");
}

float SparkBase::GetHallSensorSampleRate()
{
    auto result = ReadParameter(Parameter::kHallSensorSampleRate);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for HallSensorSampleRate");
}

uint16_t SparkBase::GetHallSensorAverageDepth()
{
    auto result = ReadParameter(Parameter::kHallSensorAverageDepth);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for HallSensorAverageDepth");
}

float SparkBase::GetSmartMotionMaxVelocity(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSmartMotionMaxVelocity_0, Parameter::kSmartMotionMaxVelocity_1, Parameter::kSmartMotionMaxVelocity_2, Parameter::kSmartMotionMaxVelocity_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SmartMotionMaxVelocity");
}

float SparkBase::GetSmartMotionMaxAccel(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSmartMotionMaxAccel_0, Parameter::kSmartMotionMaxAccel_1, Parameter::kSmartMotionMaxAccel_2, Parameter::kSmartMotionMaxAccel_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SmartMotionMaxAccel");
}

float SparkBase::GetSmartMotionMinVelOutput(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSmartMotionMinVelOutput_0, Parameter::kSmartMotionMinVelOutput_1, Parameter::kSmartMotionMinVelOutput_2, Parameter::kSmartMotionMinVelOutput_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SmartMotionMinVelOutput");
}

float SparkBase::GetSmartMotionAllowedClosedLoopError(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSmartMotionAllowedClosedLoopError_0, Parameter::kSmartMotionAllowedClosedLoopError_1, Parameter::kSmartMotionAllowedClosedLoopError_2, Parameter::kSmartMotionAllowedClosedLoopError_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SmartMotionAllowedClosedLoopError");
}

float SparkBase::GetSmartMotionAccelStrategy(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSmartMotionAccelStrategy_0, Parameter::kSmartMotionAccelStrategy_1, Parameter::kSmartMotionAccelStrategy_2, Parameter::kSmartMotionAccelStrategy_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for SmartMotionAccelStrategy");
}

float SparkBase::GetIMaxAccum(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kIMaxAccum_0, Parameter::kIMaxAccum_1, Parameter::kIMaxAccum_2, Parameter::kIMaxAccum_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for IMaxAccum");
}

float SparkBase::GetSlot3Placeholder1(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSlot3Placeholder1_0, Parameter::kSlot3Placeholder1_1, Parameter::kSlot3Placeholder1_2, Parameter::kSlot3Placeholder1_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for Slot3Placeholder1");
}

float SparkBase::GetSlot3Placeholder2(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSlot3Placeholder2_0, Parameter::kSlot3Placeholder2_1, Parameter::kSlot3Placeholder2_2, Parameter::kSlot3Placeholder2_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for Slot3Placeholder2");
}

float SparkBase::GetSlot3Placeholder3(uint8_t slot)
{
    static const std::array<Parameter, 4> params = {Parameter::kSlot3Placeholder3_0, Parameter::kSlot3Placeholder3_1, Parameter::kSlot3Placeholder3_2, Parameter::kSlot3Placeholder3_3};
    if (slot >= params.size())
    {
        throw std::out_of_range("Invalid slot number. Max value is 3.");
    }

    auto result = ReadParameter(params[slot]);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for Slot3Placeholder3");
}

float SparkBase::GetAnalogPositionConversion()
{
    auto result = ReadParameter(Parameter::kAnalogPositionConversion);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for AnalogPositionConversion");
}

float SparkBase::GetAnalogVelocityConversion()
{
    auto result = ReadParameter(Parameter::kAnalogVelocityConversion);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for AnalogVelocityConversion");
}

uint16_t SparkBase::GetAnalogAverageDepth()
{
    auto result = ReadParameter(Parameter::kAnalogAverageDepth);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for AnalogAverageDepth");
}

uint8_t SparkBase::GetAnalogSensorMode()
{
    auto result = ReadParameter(Parameter::kAnalogSensorMode);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for AnalogSensorMode");
}

bool SparkBase::GetAnalogInverted()
{
    auto result = ReadParameter(Parameter::kAnalogInverted);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for AnalogInverted");
}

uint16_t SparkBase::GetAnalogSampleDelta()
{
    auto result = ReadParameter(Parameter::kAnalogSampleDelta);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for AnalogSampleDelta");
}

uint8_t SparkBase::GetDataPortConfig()
{
    auto result = ReadParameter(Parameter::kDataPortConfig);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for DataPortConfig");
}

uint16_t SparkBase::GetAltEncoderCountsPerRev()
{
    auto result = ReadParameter(Parameter::kAltEncoderCountsPerRev);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint16_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for AltEncoderCountsPerRev");
}

uint8_t SparkBase::GetAltEncoderAverageDepth()
{
    auto result = ReadParameter(Parameter::kAltEncoderAverageDepth);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for AltEncoderAverageDepth");
}

uint8_t SparkBase::GetAltEncoderSampleDelta()
{
    auto result = ReadParameter(Parameter::kAltEncoderSampleDelta);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for AltEncoderSampleDelta");
}

bool SparkBase::GetAltEncoderInverted()
{
    auto result = ReadParameter(Parameter::kAltEncoderInverted);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for AltEncoderInverted");
}

float SparkBase::GetAltEncoderPositionFactor()
{
    auto result = ReadParameter(Parameter::kAltEncoderPositionFactor);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for AltEncoderPositionFactor");
}

float SparkBase::GetAltEncoderVelocityFactor()
{
    auto result = ReadParameter(Parameter::kAltEncoderVelocityFactor);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for AltEncoderVelocityFactor");
}

float SparkBase::GetDutyCyclePositionFactor()
{
    auto result = ReadParameter(Parameter::kDutyCyclePositionFactor);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for DutyCyclePositionFactor");
}

float SparkBase::GetDutyCycleVelocityFactor()
{
    auto result = ReadParameter(Parameter::kDutyCycleVelocityFactor);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for DutyCycleVelocityFactor");
}

bool SparkBase::GetDutyCycleInverted()
{
    auto result = ReadParameter(Parameter::kDutyCycleInverted);
    if (std::holds_alternative<bool>(result))
    {
        return std::get<bool>(result);
    }
    throw std::runtime_error("Invalid type returned for DutyCycleInverted");
}

uint8_t SparkBase::GetDutyCycleAverageDepth()
{
    auto result = ReadParameter(Parameter::kDutyCycleAverageDepth);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for DutyCycleAverageDepth");
}

uint8_t SparkBase::GetDutyCyclePrescalar()
{
    auto result = ReadParameter(Parameter::kDutyCyclePrescalar);
    if (std::holds_alternative<uint32_t>(result))
    {
        return static_cast<uint8_t>(std::get<uint32_t>(result));
    }
    throw std::runtime_error("Invalid type returned for DutyCyclePrescalar");
}

float SparkBase::GetDutyCycleZeroOffset()
{
    auto result = ReadParameter(Parameter::kDutyCycleZeroOffset);
    if (std::holds_alternative<float>(result))
    {
        return std::get<float>(result);
    }
    throw std::runtime_error("Invalid type returned for DutyCycleZeroOffset");
}
