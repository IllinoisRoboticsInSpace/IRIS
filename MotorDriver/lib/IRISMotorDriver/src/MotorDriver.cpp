#include "MotorDriver.h"
#include "DebugTools.h"

EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE> MotorDriver::send_command_buffer = EmbeddedProto::WriteBufferFixedSize<SEND_COMMAND_BUFFER_SIZE>();

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> configs)
    : serialTransferBaudRate(serialTransferBaudRate), configs(configs), debug_mode_enabled(false)
{
    
}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
    : serialTransferBaudRate(serialTransferBaudRate), configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>()), debug_mode_enabled(false)
{
    
}

MotorDriver::MotorDriver()
    : serialTransferBaudRate(DEFAULT_HOST_SERIAL_BAUD_RATE), configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>()), debug_mode_enabled(false)
{

}

/**
 * Initialize motor driver communication lines and supporting devices
*/
bool MotorDriver::initMotorDriver()
{
    // Check if enabled configs need initialization.
    for (SabertoothOperator sabertoothOperator : configs)
    {
        // Configs can be set and enabled before serial connection is made
        // Otherwise configs can only become enabled by host driver.
        if (sabertoothOperator.getEnabled() == true)
        {
            sabertoothOperator.init();
        }
    }
    Serial.begin(serialTransferBaudRate); //Serial used for USB is reserved for communication with host
    while (!Serial) {} //Wait till connection to host is made
    DEBUG_PRINTLN("Initialized Motor Driver")
    return true;
}

SabertoothOperator MotorDriver::getConfig(unsigned int motorID)
{
    return configs[motorID];
}

void MotorDriver::setConfig(unsigned int motorID, SabertoothOperator config)
{
    configs[motorID] = config;  // Does not init with new config init motor driver must be called again.
}

void MotorDriver::resetConfigs()
{
    // Maybe in future maintain the type of the config
    configs = std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>();
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
            if (configs[motorID].getEnabled() == true)
            {
                configs[motorID].setOutput(turn_motor.get_percentOutput());
            }
            break;
        }
        case Opcode_To_Arduino::STOP_ALL_MOTORS:
        {
            // Maybe make special stop function in operator
            for (SabertoothOperator config : configs)
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
            auto config_update = deserialized_message.get_configData();
            int motorID = config_update.get_motorID();
            bool error = configs[motorID].applyConfigUpdate(config_update);
            break;
        }
        case Opcode_To_Arduino::SET_DEBUG_MODE:
        {
            debug_mode_enabled = deserialized_message.get_debugMode().get_enabled();
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

    // Write entire message at once
    int message_bytes = send_command_buffer.get_size();

    int bytes_left_to_write = message_bytes;
    // int bytes_written = 
    while (bytes_left_to_write > 0)
    {
        Serial.print("AAAA");
        // Get currently available bytes in serial to write to
        int available_serial_bytes = Serial.availableForWrite();
        
        // Don't write more than available serial bytes
        int bytes_to_write = bytes_left_to_write;
        if (bytes_to_write > available_serial_bytes)
        {
            bytes_to_write = available_serial_bytes;
        }

        // Get the starting pointer of the bytes left to write
        int pointer_offset = message_bytes - bytes_left_to_write;
        uint8_t* pointer = send_command_buffer.get_data();
        Serial.write(pointer + pointer_offset, bytes_to_write);
        bytes_left_to_write = bytes_left_to_write - bytes_to_write;

        // Debug
        Serial.print("BBBB");
    }

    return true;
}
