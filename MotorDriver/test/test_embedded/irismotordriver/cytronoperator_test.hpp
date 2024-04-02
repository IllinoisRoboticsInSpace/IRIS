#include "unity.h"
#include "CytronOperator.h"

void cytron_operator_default_constructor_test(void)
{
    CytronOperator default_operator;
    TEST_ASSERT_FALSE(default_operator.getEnabled());
}

void cytron_operator_require_disabled_for_update_test(void) //breaks here, dosen't fail, just stops working
{
    CytronOperator default_operator; 
    Cytron_Config_Data update_pin;
    update_pin.set_motorID(1);
    update_pin.set_pwm_pin(1);
    bool successful = default_operator.applyConfigUpdate(update_pin);
    // Allowed to apply update on disabled config
    TEST_ASSERT_TRUE(successful);

    // Update to enabled
    Cytron_Config_Data update_enabled;
    update_enabled.set_enabled(true);
    successful = default_operator.applyConfigUpdate(update_enabled);
    // Only allowed to apply enabled update regardless of previous enable status
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_TRUE(default_operator.getEnabled());

    // Fail to apply update to enabled config
    successful = default_operator.applyConfigUpdate(update_pin);
    TEST_ASSERT_FALSE(successful);

    // Sanity check
    update_enabled.set_enabled(false);
    successful = default_operator.applyConfigUpdate(update_enabled);
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_FALSE(default_operator.getEnabled());

    successful = default_operator.applyConfigUpdate(update_pin);
    TEST_ASSERT_TRUE(successful);
}

void cytron_operator_update_key_not_set_test(void)
{
    CytronOperator default_operator;
    default_operator.setEnabled(true);
    Cytron_Config_Data empty_update;
    bool successful = default_operator.applyConfigUpdate(empty_update);
    TEST_ASSERT_FALSE(successful);
}
