#include "unity.h"
#include "SabertoothOperator.h"

void sabertooth_operator_default_constructor_test(void)
{
    SabertoothOperator default_operator;
    TEST_ASSERT_FALSE(default_operator.getEnabled());
}

void require_disabled_for_update_test(void)
{
    SabertoothOperator default_operator; // Enabled is false
    Sabertooth_Config_Data update_address;
    update_address.set_address(130);
    bool successful = default_operator.applyConfigUpdate(update_address);
    // Allowed to apply update on disabled config
    TEST_ASSERT_TRUE(successful);

    // Update to enabled
    Sabertooth_Config_Data update_enabled;
    update_enabled.set_enabled(true);
    successful = default_operator.applyConfigUpdate(update_enabled);
    // Only allowed to apply enabled update regardless of previous enable status
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_TRUE(default_operator.getEnabled());

    // Fail to apply update to enabled config
    successful = default_operator.applyConfigUpdate(update_address);
    TEST_ASSERT_FALSE(successful);

    // Sanity check
    update_enabled.set_enabled(false);
    successful = default_operator.applyConfigUpdate(update_enabled);
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_FALSE(default_operator.getEnabled());

    successful = default_operator.applyConfigUpdate(update_address);
    TEST_ASSERT_TRUE(successful);
}

void update_key_not_set_test(void)
{
    SabertoothOperator default_operator;
    default_operator.setEnabled(true);
    Sabertooth_Config_Data empty_update;
    bool successful = default_operator.applyConfigUpdate(empty_update);
    TEST_ASSERT_FALSE(successful);
}
