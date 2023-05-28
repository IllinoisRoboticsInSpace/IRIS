#include "unity.h"

void example_passing_test(void)
{
    //The messages are messages that are displayed on failure
    TEST_ASSERT_EQUAL_INT32_MESSAGE(2, 2, "This is meant to be a passing test");
}

void example_failing_test(void)
{
    TEST_ASSERT_EQUAL_INT32_MESSAGE(1, 2, "This is meant to be a failing test");
}