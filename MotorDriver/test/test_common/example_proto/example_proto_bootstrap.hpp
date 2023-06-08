#include "proto_types_test.hpp"

void RUN_EXAMPLE_PROTO_TESTS(void)
{
    RUN_TEST(basic_types_test);
    RUN_TEST(nested_message_test);
    RUN_TEST(repeated_message_test);
    RUN_TEST(string_message_test);
    RUN_TEST(bytes_message_test);
}