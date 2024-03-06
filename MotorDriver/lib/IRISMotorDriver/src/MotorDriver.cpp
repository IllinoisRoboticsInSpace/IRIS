#include "MotorDriver.h"
#include "DebugTools.h"

EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE> MotorDriver::send_command_buffer = EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE>();

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> configs)
    : serialTransferBaudRate(serialTransferBaudRate), motor_configs(configs), encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>())
    , debug_mode_enabled(false), proto_send_message(Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>())
{
    
}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
    : serialTransferBaudRate(serialTransferBaudRate), motor_configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
    , encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()), debug_mode_enabled(false)
    , proto_send_message(Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>())
{
    
}

MotorDriver::MotorDriver()
    : serialTransferBaudRate(DEFAULT_HOST_SERIAL_BAUD_RATE), motor_configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
    , encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()), debug_mode_enabled(false)
    , proto_send_message(Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>())
{

}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    // Check if enabled configs need initialization.
    for (SabertoothOperator sabertoothOperator : motor_configs)
    {
        // Configs can be set and enabled before serial connection is made
        // Otherwise configs can only become enabled by host driver.
        if (sabertoothOperator.getEnabled() == true)
        {
            sabertoothOperator.init();
        }
    }
     // Check if enabled configs need initialization.
    for (RotaryEncoderOperator rotaryEncoderOperator : encoder_configs)
    {
        // Configs can be set and enabled before serial connection is made
        // Otherwise configs can only become enabled by host driver.
        if (rotaryEncoderOperator.getEnabled() == true)
        {
            rotaryEncoderOperator.init();
        }
    }

    Serial.begin(serialTransferBaudRate); //Serial used for USB is reserved for communication with host
    while (!Serial) {} //Wait till connection to host is made
    DEBUG_PRINTLN("Initialized Motor Driver")
    return true;
}

SabertoothOperator MotorDriver::getConfig(unsigned int motorID)
{
    return motor_configs[motorID];
}

void MotorDriver::setConfig(unsigned int motorID, SabertoothOperator config)
{
    motor_configs[motorID] = config;  // Does not init with new config init motor driver must be called again.
}

void MotorDriver::resetConfigs()
{
    // Maybe in future maintain the type of the config
    motor_configs = std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>();
}

void MotorDriver::setDebugMode(bool enabled)
{
    debug_mode_enabled = enabled;
}

/**
 * Reads enough bytes for a fixed length message defined FIXED_RECEIVED_MESSAGE_LENGTH
 * Expects that parse() will be run afterwards so there will always be space in the command buffer.
 * @return number of bytes read
*/
unsigned int MotorDriver::read()
{
    int bytes = Serial.available();
    // Buffer should never become full
    if (bytes > 0)
    {
        int current_buf_size = receive_command_buffer.get_size();
        int bytes_to_read = bytes;
        // Read only enough bytes to fill message
        if ((current_buf_size + bytes) - FIXED_RECEIVED_MESSAGE_LENGTH > 0)
        {
            bytes_to_read = FIXED_RECEIVED_MESSAGE_LENGTH - current_buf_size;
        }
        // Write bytes to command_buffer
        int bytes_written = Serial.readBytes(receive_command_buffer.get_data() + current_buf_size, bytes_to_read);
        receive_command_buffer.set_bytes_written(current_buf_size + bytes_written);
        return bytes_to_read;
    }
    return 0;
}

EmbeddedProto::Error MotorDriver::parse(Serial_Message_To_Arduino& deserialized_message, EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE>& buffer)
{ 
    // Maybe add more error handling
    auto deserialize_status = deserialized_message.deserialize(buffer);
    // DeMorgan's Law
    // not (NO_ERRORS or INVALID_FIELD_ID) = (not NO_ERRORS and not INVALID_FIELD_ID)
    // If field id is zero then this means the message is zero extended and will parse correctly
    if(EmbeddedProto::Error::NO_ERRORS != deserialize_status && EmbeddedProto::Error::INVALID_FIELD_ID != deserialize_status)
    {
        DEBUG_PRINTLN("Deserialization Produced Error")
    }
    return deserialize_status;
}

void MotorDriver::execute(Serial_Message_To_Arduino& deserialized_message)
{
    Opcode_To_Arduino opcode = deserialized_message.get_opcode();
    switch (opcode)
    {
        case Opcode_To_Arduino::TURN_MOTOR:
        {
            auto turn_motor = deserialized_message.get_motorCommand();
            int motorID = turn_motor.get_motorID();
            if (motor_configs[motorID].getEnabled() == true)
            {
                motor_configs[motorID].setOutput(turn_motor.get_percentOutput());
            }
            break;
        }
        case Opcode_To_Arduino::STOP_ALL_MOTORS:
        {
            // Maybe make special stop function in operator
            for (SabertoothOperator config : motor_configs)
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
            auto config_update = deserialized_message.get_sabertoothConfigData();
            int motorID = config_update.get_motorID();
            bool error = motor_configs[motorID].applyConfigUpdate(config_update);
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
            // Maybe place in some preallocated memory in MotorDriver class
            Encoder_Count encoder_count;
            int encoderID = deserialized_message.get_encoderCountRequest().get_encoderID();
            encoder_count.set_encoderID(encoderID);
            encoder_count.set_tick_count(encoder_configs[encoderID].get_encoder_tick_count());
            proto_send_message.set_encoder_count_data(encoder_count);
            send_message(proto_send_message);
            break;
        }
        //Impossible to have invalid opcode unless deserialization did not work.
        default:
            DEBUG_PRINTLN("Impossible OPCODE!")
            break;
    }
}

/**
 * Run motor driver update loop
*/
void MotorDriver::update()
{
    unsigned int bytes_read = read(); // Places serial data into command buffer
    
    if ((bytes_read != 0) && (receive_command_buffer.get_size() == FIXED_RECEIVED_MESSAGE_LENGTH))
    {
        Serial_Message_To_Arduino message;
        auto parse_error_status = parse(message, receive_command_buffer);
        execute(message);
        if (debug_mode_enabled == true) // Send back data on debug mode on
        {
            Serial.write(receive_command_buffer.get_data(), receive_command_buffer.get_size());
        }
        receive_command_buffer.clear();
    }
}

/**
 * This function will stay run in loop until the entire message is sent. This is a problem in terms of timing because
 * Serial data transfer is much slower than the CPU, so the CPU will be stalling until the transfer finished.
*/
bool MotorDriver::send_message(const Serial_Message_To_Jetson<MAX_DEBUG_STRING_SIZE_BYTES>& message_to_jetson)
{
    // Make space
    send_command_buffer.clear();

    // Serialize message into buffer
    auto serialization_status = message_to_jetson.serialize(send_command_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != serialization_status)
    {
        DEBUG_PRINTLN("Serialization Produced Error");
        return false;
    }

    // Write entire message without stopping
    int message_bytes = send_command_buffer.get_size();

    int bytes_written = 0;
    while (bytes_written < SEND_COMMAND_BUFFER_SIZE)
    {
        // Get currently available bytes in serial to write to
        int available_serial_bytes = Serial.availableForWrite();
        
        // If this is true we want to print zeros till the end of the
        // max message length.
        if (bytes_written >= message_bytes)
        {
            int bytes_to_write = available_serial_bytes;
            // Enforce correct length of bytes
            if (SEND_COMMAND_BUFFER_SIZE - bytes_written < available_serial_bytes)
            {
                bytes_to_write = SEND_COMMAND_BUFFER_SIZE - bytes_written;
            }

            // Get the starting pointer of the bytes left to write
            for (int i = 0; i < bytes_to_write; i++)
            {
                Serial.write((byte)0x0);
            }
            
            bytes_written = bytes_written + bytes_to_write;
        }
        else
        {
            // Don't write more than available serial bytes
            int bytes_to_write = available_serial_bytes;
            if (message_bytes - bytes_written < available_serial_bytes)
            {
                // Set difference to end of proto buf data
                bytes_to_write = message_bytes - bytes_written;
            }

            // Get the starting pointer of the bytes left to write
            int pointer_offset = bytes_written;
            uint8_t* pointer = send_command_buffer.get_data();
            Serial.write(pointer + pointer_offset, bytes_to_write);

            bytes_written = bytes_written + bytes_to_write;
        }
    }

    return true;
}
