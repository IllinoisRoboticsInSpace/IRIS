#include "motordriver_test.hpp"
#include "sabertoothoperator_test.hpp"
#include "protobufutilities_test.hpp"

void RUN_IRISMOTORDRIVER_TESTS(void)
{
    // MotorDriver
    RUN_TEST(motor_driver_default_constructor_test);
    RUN_TEST(set_reset_config_test);
    RUN_TEST(parse_serial_test);
    RUN_TEST(execute_config_motor_message_test);

    // SabertoothOperator
    RUN_TEST(sabertooth_operator_default_constructor_test);
    RUN_TEST(require_disabled_for_update_test);
    RUN_TEST(update_key_not_set_test);

    // ProtobufUtilities
    RUN_TEST(saberooth_serial_line_conversion_test);
    RUN_TEST(latchmode_conversion_test);
}