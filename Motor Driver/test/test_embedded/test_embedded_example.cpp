#include <Arduino.h>
#include <unity.h>

void example_passing_test(void)
{
    //The messages are failure messages
    TEST_ASSERT_EQUAL_INT32_MESSAGE(2, 2, "This is meant to be a passing test but somehow failed");
}

void example_failing_test(void)
{
    TEST_ASSERT_EQUAL_INT32_MESSAGE(1, 2, "This is a failing test");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // NOTE!!! Wait for ~>2 secs
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(example_passing_test);
    //RUN_TEST(example_failing_test);
    UNITY_END();
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}