#include <string.h> //This include fixes memcpy not declared error in following include statements
#include <stdio.h>

#include "WriteBufferFixedSize.h"
#include "ReadBufferFixedSize.h"
#include "generated/example.h"
#include "unity.h"

static EmbeddedProto::WriteBufferFixedSize<256> write_fixed_buffer;
static EmbeddedProto::ReadBufferFixedSize<256> read_fixed_buffer;
// static char unity_message_buffer[256]; Reusing the same string buffer is efficient if needed.

void reset(void)
{
    write_fixed_buffer.clear();
    read_fixed_buffer.clear();
}

void basic_types_test(void)
{
    reset();

    Types message;
    message.set_unsigned_integer_32(1);
    message.set_signed_integer_32(-1);
    message.set_unsigned_integer_64(100);
    message.set_signed_integer_64(-100);
    message.set_bool_t(true);
    message.set_float_t(1.5);
    message.set_double_t(-10.5);
    TEST_ASSERT_EQUAL_UINT32(1, message.get_unsigned_integer_32());
    TEST_ASSERT_EQUAL_INT32(-1, message.get_signed_integer_32());
    TEST_ASSERT_EQUAL_UINT64(100, message.get_unsigned_integer_64());
    TEST_ASSERT_EQUAL_INT64(-100, message.get_signed_integer_64());
    TEST_ASSERT_TRUE(message.get_bool_t());
    TEST_ASSERT_EQUAL_FLOAT(1.5, message.get_float_t());
    TEST_ASSERT_EQUAL_DOUBLE(-10.5, message.get_double_t());

    uint32_t size = message.serialized_size();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, size);

    // TODO: Investigate Enabling Printing from Tests
    // Printing should print to the vscode terminal
    // TEST_MESSAGE("Bruh\n");
    // TEST_PRINTF("Made it here %d", 10);

    // Serialize Message
    auto serialization_status = message.serialize(write_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != serialization_status)
    {
        TEST_FAIL_MESSAGE("Serialization Produced Error");
    }
    uint8_t* write_data_ptr = write_fixed_buffer.get_data();
    uint32_t write_data_size = write_fixed_buffer.get_size();
    TEST_ASSERT_EQUAL_INT32(size, write_data_size);

    // Deserialize Message
    uint8_t* read_data_ptr = read_fixed_buffer.get_data();
    memcpy(read_data_ptr, write_data_ptr, write_data_size);
    read_fixed_buffer.set_bytes_written(write_data_size);

    Types deserialized_message;
    auto deserialize_status = deserialized_message.deserialize(read_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != deserialize_status)
    {
        TEST_FAIL_MESSAGE("Deserialization Produced Error");
    }
    TEST_ASSERT_EQUAL_UINT32(1, deserialized_message.get_unsigned_integer_32());
    TEST_ASSERT_EQUAL_INT32(-1, deserialized_message.get_signed_integer_32());
    TEST_ASSERT_EQUAL_UINT64(100, deserialized_message.get_unsigned_integer_64());
    TEST_ASSERT_EQUAL_INT64(-100, deserialized_message.get_signed_integer_64());
    TEST_ASSERT_TRUE(deserialized_message.get_bool_t());
    TEST_ASSERT_EQUAL_FLOAT(1.5, deserialized_message.get_float_t());
    TEST_ASSERT_EQUAL_DOUBLE(-10.5, deserialized_message.get_double_t());
}

void nested_message_test(void)
{
    reset();

    Nested message;
    A nested_message;
    nested_message.set_a(1);
    message.set_nested_a(nested_message);
    TEST_ASSERT_EQUAL_INT32(1, message.get_nested_a().get_a());
    // Accessing the nested message and editing it.
    message.mutable_nested_a().set_a(2);
    TEST_ASSERT_EQUAL_INT32(2, message.get_nested_a().get_a());

    uint32_t size = message.serialized_size();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, size);

    // Serialize Message
    auto serialization_status = message.serialize(write_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != serialization_status)
    {
        TEST_FAIL_MESSAGE("Serialization Produced Error");
    }
    uint8_t* write_data_ptr = write_fixed_buffer.get_data();
    uint32_t write_data_size = write_fixed_buffer.get_size();
    TEST_ASSERT_EQUAL_INT32(size, write_data_size);

    // Deserialize Message
    uint8_t* read_data_ptr = read_fixed_buffer.get_data();
    memcpy(read_data_ptr, write_data_ptr, write_data_size);
    read_fixed_buffer.set_bytes_written(write_data_size);

    Nested deserialized_message;
    auto deserialize_status = deserialized_message.deserialize(read_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != deserialize_status)
    {
        TEST_FAIL_MESSAGE("Deserialization Produced Error");
    }
    TEST_ASSERT_EQUAL_INT32(2, deserialized_message.get_nested_a().get_a());
}

void repeated_message_test(void)
{
    reset();

    Repeated message;

    for(int i = 0; i < 10; i++)
    {
        message.add_repeated_int(i);
    }

    TEST_ASSERT_EQUAL_UINT32(10, message.get_repeated_int().get_length());

    // Check
    for(int i = 0; i < 10; i++)
    {
        TEST_ASSERT_EQUAL_INT32(i, message.get_repeated_int()[i]);
    }

    // Set index values
    message.set_repeated_int(0, 2);
    TEST_ASSERT_EQUAL_INT32(2, message.get_repeated_int()[0]);
    // Set back to 0
    message.set_repeated_int(0, 0);

    // Similar way to edit data using references
    EmbeddedProto::int32& mutable_reference = message.mutable_repeated_int(0);
    mutable_reference = 2;
    TEST_ASSERT_EQUAL_INT32(2, message.get_repeated_int()[0]);
    // Set back to 0
    message.set_repeated_int(0, 0);
    
    uint32_t size = message.serialized_size();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, size);

    // Serialize Message
    auto serialization_status = message.serialize(write_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != serialization_status)
    {
        TEST_FAIL_MESSAGE("Serialization Produced Error");
    }
    uint8_t* write_data_ptr = write_fixed_buffer.get_data();
    uint32_t write_data_size = write_fixed_buffer.get_size();
    TEST_ASSERT_EQUAL_INT32(size, write_data_size);

    // Deserialize Message
    uint8_t* read_data_ptr = read_fixed_buffer.get_data();
    memcpy(read_data_ptr, write_data_ptr, write_data_size);
    read_fixed_buffer.set_bytes_written(write_data_size);

    Repeated deserialized_message;
    auto deserialize_status = deserialized_message.deserialize(read_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != deserialize_status)
    {
        TEST_FAIL_MESSAGE("Deserialization Produced Error");
    }
    for(int i = 0; i < 10; i++)
    {
        TEST_ASSERT_EQUAL_INT32(i, message.get_repeated_int()[i]);
    }
}

void string_message_test(void)
{
    reset();

    Text<128> message;
    
    char test_message[] = "String Message Test!";
    message.mutable_text_data() = test_message;

    TEST_ASSERT_EQUAL_STRING(test_message, message.text_data());
    
    uint32_t size = message.serialized_size();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, size);

    uint32_t string_size = message.get_text_data().get_length();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0, string_size);

    // Serialize Message
    auto serialization_status = message.serialize(write_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != serialization_status)
    {
        TEST_FAIL_MESSAGE("Serialization Produced Error");
    }
    uint8_t* write_data_ptr = write_fixed_buffer.get_data();
    uint32_t write_data_size = write_fixed_buffer.get_size();
    TEST_ASSERT_EQUAL_INT32(size, write_data_size);

    // Deserialize Message
    uint8_t* read_data_ptr = read_fixed_buffer.get_data();
    memcpy(read_data_ptr, write_data_ptr, write_data_size);
    read_fixed_buffer.set_bytes_written(write_data_size);

    Text<256> deserialized_message;
    auto deserialize_status = deserialized_message.deserialize(read_fixed_buffer);
    if(::EmbeddedProto::Error::NO_ERRORS != deserialize_status)
    {
        TEST_FAIL_MESSAGE("Deserialization Produced Error");
    }
    TEST_ASSERT_EQUAL_STRING(test_message, deserialized_message.text_data());
}