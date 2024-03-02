#include "MotorDriver.h"
#include "DebugTools.h"

/**
 * Initialize motor driver state
*/
MotorDriver::MotorDriver(unsigned int serialTransferBaudRate,  std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> Sabertooth_configs, std::array<PIDHandler, MAX_PID_CONGIFS> PID_configs)
    : serialTransferBaudRate(serialTransferBaudRate), motor_configs(Sabertooth_configs), pid_configs(PID_configs), debug_mode_enabled(false), encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()) {}

MotorDriver::MotorDriver(unsigned int serialTransferBaudRate)
    : serialTransferBaudRate(serialTransferBaudRate), motor_configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
    , encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()), debug_mode_enabled(false)
{
    
}

MotorDriver::MotorDriver()
    : serialTransferBaudRate(DEFAULT_HOST_SERIAL_BAUD_RATE), motor_configs(std::array<SabertoothOperator, MAX_MOTOR_CONFIGS>())
    , encoder_configs(std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS>()), debug_mode_enabled(false)
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
        return bytes_to_read;
    }
    return 0;
}

EmbeddedProto::Error MotorDriver::parse(Serial_Message_To_Arduino& deserialized_message, EmbeddedProto::ReadBufferFixedSize<COMMAND_BUFFER_SIZE>& buffer)
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
        case Opcode_To_Arduino::CONFIG_PID:
        {
            auto config_update = deserialized_message.get_pidConfigData();
            int pidID = config_update.get_PID_ID();
            bool error = pid_configs[pidID].applyConfigUpdate(config_update);

            break;
        }
        case Opcode_To_Arduino::SET_PID_SETPOINT:
        {
            auto set_point_command = deserialized_message.get_setPIDSetpoint();
            int pidID = set_point_command.get_PID_ID();
            bool error = pid_configs[pidID].applySetPoint(set_point_command);
            break;
        }
        case Opcode_To_Arduino::SET_PID_CONTROL:
        {
            auto set_pid_command = deserialized_message.get_setPIDControl();
            int pidID = set_pid_command.get_PID_ID();
            bool error = pid_configs[pidID].applyMotorControl(set_pid_command);
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
        //Impossible to have invalid opcode unless deserialization did not work.
        default:
        {
            DEBUG_PRINTLN("Impossible OPCODE!")
            break;
        }
    }
}
/**
 * Run motor driver update loop
*/
void MotorDriver::update(){
    //FUTURE:
    // read encoder data (OTHER BRANCH)
    // run PID loops DONE (NOT TESETD)
    // give motors pid output DONE (NOT TESTED)

    for(auto pid : pid_configs){
        pid.update_pid(0.0); //change the input to the encoder output
        if(pid.get_in_control()){
            motor_configs[pid.get_motorID()].setOutput(pid.get_motor_value());
        }
    }

    unsigned int bytes_read = read(); // Places serial data into command buffer
    
    if ((bytes_read != 0) && (command_buffer.get_size() == FIXED_RECEIVED_MESSAGE_LENGTH))
    {
        Serial_Message_To_Arduino message;
        auto parse_error_status = parse(message, command_buffer);
        execute(message);
        if (debug_mode_enabled == true) // Send back data on debug mode on
        {
            Serial.write(command_buffer.get_data(), command_buffer.get_size());
        }
        command_buffer.clear();
    }
}
