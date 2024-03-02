#include "unity.h"
#include "RotaryEncoderOperator.h"

void rotary_encoder_operator_default_constructor_test(void)
{
    RotaryEncoderOperator default_operator;
    TEST_ASSERT_FALSE(default_operator.getEnabled());
}

void rotary_encoder_operator_require_disabled_for_update_test(void)
{
    RotaryEncoderOperator default_operator; // Enabled is false
    Encoder_Config_Data update_pin1;
    update_pin1.set_pinIn(10);
    bool successful = default_operator.applyConfigUpdate(update_pin1);
    // Allowed to apply update on disabled config
    TEST_ASSERT_TRUE(successful);

    // Update to enabled
    Encoder_Config_Data update_enabled;
    update_enabled.set_enabled(true);
    successful = default_operator.applyConfigUpdate(update_enabled);
    // Only allowed to apply enabled update regardless of previous enable status
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_TRUE(default_operator.getEnabled());

    // Fail to apply update to enabled config
    successful = default_operator.applyConfigUpdate(update_pin1);
    TEST_ASSERT_FALSE(successful);

    // Sanity check
    update_enabled.set_enabled(false);
    successful = default_operator.applyConfigUpdate(update_enabled);
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_FALSE(default_operator.getEnabled());

    successful = default_operator.applyConfigUpdate(update_pin1);
    TEST_ASSERT_TRUE(successful);
}

void rotary_encoder_operator_update_key_not_set_test(void)
{
    RotaryEncoderOperator default_operator;
    default_operator.setEnabled(true);
    Encoder_Config_Data empty_update;
    bool successful = default_operator.applyConfigUpdate(empty_update);
    TEST_ASSERT_FALSE(successful);
}