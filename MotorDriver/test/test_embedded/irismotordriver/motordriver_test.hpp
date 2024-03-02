#include "unity.h"
#include "MotorDriver.h"

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
    EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE> command_buffer;

    Serial_Message_To_Arduino received_message;
    Sabertooth_Config_Data sabertooth_config_update;
    sabertooth_config_update.set_enabled(true);
    sabertooth_config_update.set_motorID(0);

    received_message.set_configData(sabertooth_config_update);
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
    EmbeddedProto::ReadBufferFixedSize<RECEIVED_COMMAND_BUFFER_SIZE> command_buffer;

    Serial_Message_To_Arduino received_message;
    Sabertooth_Config_Data sabertooth_config_update;
    sabertooth_config_update.set_enabled(true);
    const int motorID = 0;
    sabertooth_config_update.set_motorID(motorID);

    received_message.set_configData(sabertooth_config_update);
    received_message.set_opcode(Opcode_To_Arduino::CONFIG_MOTOR);

    TEST_ASSERT_FALSE(default_motor_driver.getConfig(motorID).getEnabled());

    default_motor_driver.execute(received_message);
    TEST_ASSERT_TRUE(default_motor_driver.getConfig(motorID).getEnabled());
}
