#include "motordriver_test.hpp"

void RUN_IRISMOTORDRIVER_TESTS(void)
{
    RUN_TEST(default_constructor_test);
    RUN_TEST(set_reset_config_test);
    RUN_TEST(parse_serial_test);
    RUN_TEST(execute_config_motor_message_test);
}