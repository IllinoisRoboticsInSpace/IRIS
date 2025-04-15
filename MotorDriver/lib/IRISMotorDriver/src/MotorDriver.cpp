#include "MotorDriver.h"
#include "DebugTools.h"

// Initialize the static send buffer.
EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE> MotorDriver::send_command_buffer =
    EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE>();

// Constructors (only showing one for brevity; add similar initialization for cytron_configs as needed)
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate,
                         std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> configs)
    : serialTransferBaudRate(serialTransferBaudRate),
      motor_configs(configs),
      // Initialize the cytron configs with their default constructor.
      cytron_configs(std::array<CytronOperator, MAX_MOTOR_CONFIGS>()),
      encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()),
      debug_mode_enabled(false),
      proto_send_message(Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>())
{
}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
    : serialTransferBaudRate(serialTransferBaudRate),
      motor_configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>()),
      cytron_configs(std::array<CytronOperator, MAX_MOTOR_CONFIGS>()),
      encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()),
      debug_mode_enabled(false),
      proto_send_message(Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>())
{
}

MotorDriver::MotorDriver()
    : serialTransferBaudRate(DEFAULT_HOST_SERIAL_BAUD_RATE),
      motor_configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>()),
      cytron_configs(std::array<CytronOperator, MAX_MOTOR_CONFIGS>()),
      encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()),
      debug_mode_enabled(false),
      proto_send_message(Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>())
{
}

// Initialize all motor and encoder configurations.
bool MotorDriver::initMotorDriver()
{
    // Initialize Sabertooth operators.
    for (SabertoothOperator sabertoothOperator : motor_configs)
    {
        if (sabertoothOperator.getEnabled() == true)
        {
            sabertoothOperator.init();
        }
    }
    
    // Initialize Cytron operators.
    for (CytronOperator cytronOperator : cytron_configs)
    {
        if (cytronOperator.getEnabled() == true)
        {
            cytronOperator.init();
        }
    }

    // Initialize rotary encoder operators.
    for (RotaryEncoderOperator rotaryEncoderOperator : encoder_configs)
    {
        if (rotaryEncoderOperator.getEnabled() == true)
        {
            rotaryEncoderOperator.init();
        }
    }

    Serial.begin(serialTransferBaudRate); // USB serial for host communication.
    while (!Serial) {} // Wait for host connection.
    delay(1000);
    DEBUG_PRINTLN("Initialized Motor Driver");
    return true;
}

SabertoothOperator MotorDriver::getConfig(unsigned int motorID)
{
    return motor_configs[motorID];
}

void MotorDriver::setConfig(unsigned int motorID, SabertoothOperator config)
{
    motor_configs[motorID] = config;
}

CytronOperator MotorDriver::getCytronConfig(unsigned int motorID)
{
    return cytron_configs[motorID];
}

void MotorDriver::setCytronConfig(unsigned int motorID, CytronOperator config)
{
    cytron_configs[motorID] = config;
}

void MotorDriver::resetConfigs()
{
    motor_configs = std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>();
    // Optionally, also reset cytron_configs if desired.
    cytron_configs = std::array<CytronOperator, MAX_MOTOR_CONFIGS>();
}

void MotorDriver::setDebugMode(bool enabled)
{
    debug_mode_enabled = enabled;
}

// Read incoming serial data.
unsigned int MotorDriver::read()
{
    int bytes = Serial.available();
    if (bytes > 0)
    {
        int current_buf_size = receive_command_buffer.get_size();
        int bytes_to_read = bytes;
        if ((current_buf_size + bytes) - FIXED_RECEIVED_MESSAGE_LENGTH > 0)
        {
            bytes_to_read = FIXED_RECEIVED_MESSAGE_LENGTH - current_buf_size;
        }
        int bytes_written = Serial.readBytes(receive_command_buffer.get_data() + current_buf_size, bytes_to_read);
        receive_command_buffer.set_bytes_written(current_buf_size + bytes_written);
        return bytes_to_read;
    }
    return 0;
}

EmbeddedProto::Error MotorDriver::parse(Serial_Message_To_Arduino& deserialized_message,
                                          EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE>& buffer)
{ 
    auto deserialize_status = deserialized_message.deserialize(buffer);
    if(EmbeddedProto::Error::NO_ERRORS != deserialize_status &&
       EmbeddedProto::Error::INVALID_FIELD_ID != deserialize_status)
    {
        DEBUG_PRINTLN("Deserialization Produced Error");
    }
    return deserialize_status;
}

// Execute the deserialized message.
void MotorDriver::execute(Serial_Message_To_Arduino& deserialized_message)
{
    Opcode_To_Arduino opcode = deserialized_message.get_opcode();
    switch (opcode)
    {
        case Opcode_To_Arduino::TURN_MOTOR:
        {
            // Existing command for Sabertooth.
            auto turn_motor = deserialized_message.get_motorCommand();
            DEBUG_PRINTLN("Turn Motor Command:");
            DEBUG_PRINT_MESSAGE(turn_motor);
            DEBUG_PRINTLN("");
            int motorID = turn_motor.get_motorID();
            if (motor_configs[motorID].getEnabled() == true)
            {
                motor_configs[motorID].setOutput(turn_motor.get_percentOutput());
            }
            break;
        }
        // NEW: Command to turn a Cytron motor.
        case Opcode_To_Arduino::TURN_CYTRON_MOTOR:
        {
            // Assumes a similar command message structure exists.
            auto turn_motor = deserialized_message.get_cytronMotorCommand();
            DEBUG_PRINTLN("Turn Cytron Motor Command:");
            DEBUG_PRINT_MESSAGE(turn_motor);
            DEBUG_PRINTLN("");
            int motorID = turn_motor.get_motorID();
            if (cytron_configs[motorID].getEnabled() == true)
            {
                cytron_configs[motorID].setOutput(turn_motor.get_percentOutput());
            }
            break;
        }
        case Opcode_To_Arduino::STOP_ALL_MOTORS:
        {
            // Stop all Sabertooth motors.
            for (SabertoothOperator config : motor_configs)
            {
                if (config.getEnabled())
                {
                    config.setOutput(0);
                }
            }
            // NEW: Stop all Cytron motors.
            for (CytronOperator config : cytron_configs)
            {
                if (config.getEnabled())
                {
                    config.setOutput(0);
                }
            }
            break;
        }
        case Opcode_To_Arduino::CONFIG_MOTOR:
        {
            // Configuration update for Sabertooth motor.
            auto config_update = deserialized_message.get_sabertoothConfigData();
            int motorID = config_update.get_motorID();
            bool error = motor_configs[motorID].applyConfigUpdate(config_update);
            break;
        }
        // NEW: Configuration update for Cytron motor.
        case Opcode_To_Arduino::CONFIG_CYTRON:
        {
            auto config_update = deserialized_message.get_cytronConfigData();
            int motorID = config_update.get_motorID();
            bool error = cytron_configs[motorID].applyConfigUpdate(config_update);
            break;
        }
        case Opcode_To_Arduino::SET_DEBUG_MODE:
        {
            debug_mode_enabled = deserialized_message.get_debugMode().get_enabled();
            break;
        }
        case Opcode_To_Arduino::CONFIG_ENCODER:
        {
            auto config_update = deserialized_message.get_encoderConfigData();
            int encoderID = config_update.get_encoderID();
            bool error = encoder_configs[encoderID].applyConfigUpdate(config_update);
            break;
        }
        case Opcode_To_Arduino::ZERO_ENCODER:
        {
            int encoderID = deserialized_message.get_zeroEncoderCommand().get_encoderID();
            encoder_configs[encoderID].set_encoder_tick_count(0);
            break;
        }
        case Opcode_To_Arduino::GET_ENCODER_COUNT:
        {
            proto_send_message.set_opcode(Opcode_To_Jetson::ENCODER_COUNT);
            Encoder_Count encoder_count;
            int encoderID = deserialized_message.get_encoderCountRequest().get_encoderID();
            encoder_count.set_encoderID(encoderID);
            encoder_count.set_tick_count(encoder_configs[encoderID].get_encoder_tick_count());
            proto_send_message.set_encoder_count_data(encoder_count);
            send_message(proto_send_message);
            break;
        }
        default:
            DEBUG_PRINTLN("Impossible OPCODE!");
            break;
    }
}

// Main update loop.
void MotorDriver::update()
{
    unsigned int bytes_read = read(); // Fill command buffer
    if ((bytes_read != 0) && (receive_command_buffer.get_size() == FIXED_RECEIVED_MESSAGE_LENGTH))
    {
        Serial_Message_To_Arduino message;
        auto parse_error_status = parse(message, receive_command_buffer);
        execute(message);
        if (debug_mode_enabled == true)
        {
            Serial.write(receive_command_buffer.get_data(), receive_command_buffer.get_size());
        }
        receive_command_buffer.clear();
    }
}

// Helper function to send a message to the host.
bool MotorDriver::send_message(const Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>& message_to_jetson)
{
    send_command_buffer.clear();
    auto serialization_status = message_to_jetson.serialize(send_command_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != serialization_status)
    {
        DEBUG_PRINTLN("Serialization Produced Error");
        return false;
    }
    int message_bytes = send_command_buffer.get_size();
    int bytes_written = 0;
    while (bytes_written < SEND_COMMAND_BUFFER_SIZE)
    {
        int available_serial_bytes = Serial.availableForWrite();
        if (bytes_written >= message_bytes)
        {
            int bytes_to_write = available_serial_bytes;
            if (SEND_COMMAND_BUFFER_SIZE - bytes_written < available_serial_bytes)
            {
                bytes_to_write = SEND_COMMAND_BUFFER_SIZE - bytes_written;
            }
            for (int i = 0; i < bytes_to_write; i++)
            {
                Serial.write((byte)0x0);
            }
            bytes_written += bytes_to_write;
        }
        else
        {
            int bytes_to_write = available_serial_bytes;
            if (message_bytes - bytes_written < available_serial_bytes)
            {
                bytes_to_write = message_bytes - bytes_written;
            }
            int pointer_offset = bytes_written;
            uint8_t* pointer = send_command_buffer.get_data();
            Serial.write(pointer + pointer_offset, bytes_to_write);
            bytes_written += bytes_to_write;
        }
    }
    return true;
}
