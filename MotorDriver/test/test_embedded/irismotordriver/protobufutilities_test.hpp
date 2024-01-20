#include "unity.h"
#include "ProtobufUtilities.h"

void saberooth_serial_line_conversion_test(void)
{
    // Must be the correct reference as only one Serial1/2/3 object exists
    TEST_ASSERT_TRUE(Serial1 == getSabertoothSerial(SabertoothSerialLine::Serial1));
    TEST_ASSERT_TRUE(Serial2 == getSabertoothSerial(SabertoothSerialLine::Serial2));
    TEST_ASSERT_TRUE(Serial3 == getSabertoothSerial(SabertoothSerialLine::Serial3));
}
