#include "MotorDriver.h"
#include "DebugTools.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate, std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> configs)
    : serialTransferBaudRate(serialTransferBaudRate), configs(configs)
{

}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
    : serialTransferBaudRate(serialTransferBaudRate), configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
{
    
}

MotorDriver::MotorDriver()
    : serialTransferBaudRate(DEFAULT_HOST_SERIAL_BAUD_RATE), configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
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

/**
 * Reads enough bytes for a fixed length message defined FIXED_RECEIVED_MESSAGE_LENGTH
 * Expects that parse() will be run afterwards so there will always be space in the command buffer.
 * 
*/
void MotorDriver::read()
{
    int bytes = Serial.available();
    // Buffer should never become full
    if (bytes > 0)
    {
        int current_buf_size = command_buffer.get_size();
        int bytes_to_read = bytes;
        // Read only enough bytes to fill message
        if ((current_buf_size + bytes) - FIXED_RECEIVED_MESSAGE_LENGTH > 0)
        {
            bytes_to_read = FIXED_RECEIVED_MESSAGE_LENGTH - current_buf_size;
        }
        // Write bytes to command_buffer
        int bytes_written = Serial.readBytes(command_buffer.get_data() + current_buf_size, bytes_to_read);
        command_buffer.set_bytes_written(current_buf_size + bytes_written);
    }
}

EmbeddedProto::Error MotorDriver::parse(Serial_Message& deserialized_message, EmbeddedProto::ReadBufferFixedSize<COMMAND_BUFFER_SIZE>& buffer)
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

void MotorDriver::execute(Serial_Message& deserialized_message)
{
    Opcode opcode = deserialized_message.get_opcode();
    switch (opcode)
    {
        case Opcode::TURN_MOTOR:
        {
            auto turn_motor = deserialized_message.get_motorCommand();
            int motorID = turn_motor.get_motorID();
            configs[motorID].setOutput(turn_motor.get_percentOutput());
            break;
        }
        case Opcode::STOP_ALL_MOTORS:
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
        case Opcode::CONFIG_MOTOR:
        {
            auto config_update = deserialized_message.get_configData();
            int motorID = config_update.get_motorID();
            bool error = configs[motorID].applyConfigUpdate(config_update);
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
    //FUTURE:
    // read encoder data
    // send back encoder data
    // run PID loops

    read(); // Places serial data into command buffer
    if (command_buffer.get_size() == FIXED_RECEIVED_MESSAGE_LENGTH)
    {
        Serial_Message message;
        auto parse_error_status = parse(message, command_buffer);
        execute(message);
        command_buffer.clear();
    }
}
