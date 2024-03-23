#include "unity.h"
#include "MotorDriver.h"
#include "DebugTools.h"

void motor_driver_default_constructor_test(void)
{
    // Default all configs not enabled
    MotorDriver default_motor_driver;
    for (int i = 0; i < MAX_MOTOR_CONFIGS; i++)
    {
        TEST_ASSERT_FALSE(default_motor_driver.getConfig(i).getEnabled());
    }
}

void set_reset_config_test(void)
{
    // Test setting configs to true
    MotorDriver default_motor_driver;
    SabertoothOperator temp_config;
    for (int i = 0; i < MAX_MOTOR_CONFIGS; i++)
    {
        temp_config = SabertoothOperator();
        temp_config.setEnabled(true);
        default_motor_driver.setConfig(i, temp_config);
        TEST_ASSERT_TRUE(default_motor_driver.getConfig(i).getEnabled());
    }
    default_motor_driver.resetConfigs();
    for (int i = 0; i < MAX_MOTOR_CONFIGS; i++)
    {
        TEST_ASSERT_FALSE(default_motor_driver.getConfig(i).getEnabled());
    }
}

// Not possible to test read serial

void parse_serial_test(void)
{
    reset();

    MotorDriver default_motor_driver;
    EmbeddedProto::ReadBufferFixedSize<COMMAND_BUFFER_SIZE> command_buffer;

    Serial_Message_To_Arduino received_message;
    Sabertooth_Config_Data sabertooth_config_update;
    sabertooth_config_update.set_enabled(true);
    sabertooth_config_update.set_motorID(0);

    received_message.set_sabertoothConfigData(sabertooth_config_update);
    received_message.set_opcode(Opcode_To_Arduino::CONFIG_MOTOR);
    
    // Serialize Message
    auto serialization_status = received_message.serialize(write_fixed_buffer);

    // Copy over message
    uint8_t* write_data_ptr = write_fixed_buffer.get_data();
    uint32_t write_data_size = write_fixed_buffer.get_size();

    // Deserialize Message
    uint8_t* command_buffer_ptr = command_buffer.get_data();
    memcpy(command_buffer_ptr, write_data_ptr, write_data_size);
    command_buffer.set_bytes_written(write_data_size);

    // Parse and deserialize
    // Testing when the serialized data is not zero extended.
    auto parse_error_status = default_motor_driver.parse(received_message, command_buffer);
    TEST_ASSERT_TRUE(parse_error_status == EmbeddedProto::Error::NO_ERRORS);

    const int zeros = FIXED_RECEIVED_MESSAGE_LENGTH - write_data_size; // number of uint8_t (bytes) set to 0
    for (int i = 0; i < zeros; i++)
    {
        command_buffer.push(0);
    }

    parse_error_status = default_motor_driver.parse(received_message, command_buffer);
    TEST_ASSERT_TRUE(parse_error_status == EmbeddedProto::Error::INVALID_FIELD_ID);
}

void execute_config_motor_message_test(void)
{
    reset();

    MotorDriver default_motor_driver;
    EmbeddedProto::ReadBufferFixedSize<COMMAND_BUFFER_SIZE> command_buffer;

    Serial_Message_To_Arduino received_message;
    Sabertooth_Config_Data sabertooth_config_update;
    sabertooth_config_update.set_enabled(true);
    const int motorID = 0;
    sabertooth_config_update.set_motorID(motorID);

    received_message.set_sabertoothConfigData(sabertooth_config_update);
    received_message.set_opcode(Opcode_To_Arduino::CONFIG_MOTOR);

    TEST_ASSERT_FALSE(default_motor_driver.getConfig(motorID).getEnabled());

    default_motor_driver.execute(received_message);
    TEST_ASSERT_TRUE(default_motor_driver.getConfig(motorID).getEnabled());
}


/*
    CONFIG_MOTOR = 0;
    TURN_MOTOR = 1; 
    STOP_ALL_MOTORS = 2;
    SET_DEBUG_MODE = 3;
    CONFIG_PID = 4;
    SET_PID_CONTROL = 5;
    SET_PID_SETPOINT = 6;
    CONFIG_ENCODER = 7;
    ZERO_ENCODER = 9;
*/


void check_proto_buf_size(void){

    Serial_Message_To_Arduino received_message;
    received_message.set_opcode(Opcode_To_Arduino::CONFIG_MOTOR);

    Sabertooth_Config_Data motor_config;
    
    motor_config.set_motorID(15);
    motor_config.set_address(15);

    received_message.set_sabertoothConfigData(motor_config);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "Sabertooth_config, address");

    motor_config.clear();
    received_message.clear();
    


    motor_config.set_motorID(15);
    motor_config.set_serialLine(SabertoothSerialLine::Serial3);

    received_message.set_sabertoothConfigData(motor_config);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "Sabertooth_config, SerialLine");

    Serial_Message_To_Arduino received_message_;

    received_message = Serial_Message_To_Arduino();

    received_message_.set_opcode(Opcode_To_Arduino::TURN_MOTOR);

    Turn_Motor motor_command;
    motor_command.set_motorID(15);
    motor_command.set_percentOutput(0.9);

    received_message_.set_motorCommand(motor_command);

    std::string msg = "Turn Motor, Perecnt Output";

    msg += std::to_string(received_message_.serialized_size());

    TEST_ASSERT_TRUE_MESSAGE(received_message_.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, msg.c_str());



    received_message = Serial_Message_To_Arduino();

    received_message.set_opcode(Opcode_To_Arduino::STOP_ALL_MOTORS);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "STOP ALL");



    received_message = Serial_Message_To_Arduino();

    received_message.set_opcode(Opcode_To_Arduino::SET_DEBUG_MODE);

    Debug_Mode mode; mode.set_enabled(true); 

    received_message.set_debugMode(mode);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "SET_DEBUG_MODE");



    received_message = Serial_Message_To_Arduino();



    received_message.set_opcode(Opcode_To_Arduino::CONFIG_PID);

   PID_Config_Data pid_config; pid_config.set_PID_ID(15); pid_config.set_kp(2);

    received_message.set_pidConfigData(pid_config);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "PID_CONFIG, kp");

    pid_config.clear_kp();
    received_message = Serial_Message_To_Arduino();

    received_message.set_opcode(Opcode_To_Arduino::CONFIG_PID);

    pid_config.set_motorID(15);

    received_message.set_pidConfigData(pid_config);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "PID_CONGIF, motorID");



    received_message = Serial_Message_To_Arduino();

    received_message.set_opcode(Opcode_To_Arduino::SET_PID_SETPOINT);

   Set_PID_Setpoint pid_set; pid_set.set_PID_ID(15); pid_set.set_setPoint(100);

    received_message.set_setPIDSetpoint(pid_set);

    std::string msg_ = "PID SETPOINT" + std::to_string(received_message.serialized_size());

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, msg_.c_str());
    //TEST_ASSERT_TRUE_MESSAGE(false, std::to_string(received_message.serialized_size()).c_str()); 


    received_message = Serial_Message_To_Arduino();

    received_message.set_opcode(Opcode_To_Arduino::CONFIG_ENCODER);

    Encoder_Config_Data encoder_config; encoder_config.set_encoderID(15); encoder_config.set_pinIn(53);

    received_message.set_encoderConfigData(encoder_config);

    TEST_ASSERT_TRUE_MESSAGE(received_message.serialized_size() <= FIXED_RECEIVED_MESSAGE_LENGTH, "ENCODER CONFIG, pinIn");



}