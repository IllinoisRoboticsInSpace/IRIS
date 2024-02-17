#include "unity.h"
#include "PIDHandler.h"

void pid_handler_default_constructor_test(void){
    //default all pid configs are disabled to match sabertooth operator test
    PIDHandler default_pid_handler;
    TEST_ASSERT_FALSE(default_pid_handler.getEnabled());
}

void require_PID_disabled_for_update_test(void){
    //mirrored after sabertooth operator tests 
    PIDHandler default_pid_handler; // Enabled is false
    PID_Config_Data update_kd;
    double updated_kd = 0;
    update_kd.set_kd(0);
    bool successful = default_pid_handler.applyConfigUpdate(update_kd);
    TEST_ASSERT_FALSE(successful);

    // Update to enabled
    PID_Config_Data update_enabled;
    update_enabled.set_enabled(true);
    successful = default_pid_handler.applyConfigUpdate(update_enabled);
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_TRUE(default_pid_handler.getEnabled());

    // Apply update
    successful = default_pid_handler.applyConfigUpdate(update_kd);
    TEST_ASSERT_TRUE(successful);

    // Sanity check
    update_enabled.set_enabled(false);
    successful = default_pid_handler.applyConfigUpdate(update_enabled);
    TEST_ASSERT_TRUE(successful);
    TEST_ASSERT_FALSE(default_pid_handler.getEnabled());

    successful = default_pid_handler.applyConfigUpdate(update_kd);
    TEST_ASSERT_FALSE(successful);
}

#define assignment_test_(configtype, dataname, setto, applyfunc) configtype update_##dataname; update_##dataname.set_##dataname(setto); \
    TEST_ASSERT_TRUE(default_pid_handler.applyfunc(update_##dataname)); \
    TEST_ASSERT_TRUE(default_pid_handler.get_##dataname() == setto);

void correct_PID_message_assign_test(void){
    PIDHandler default_pid_handler;
    PID_Config_Data update_enabled;
    update_enabled.set_enabled(true);
    default_pid_handler.applyConfigUpdate(update_enabled);
    TEST_ASSERT_TRUE(default_pid_handler.getEnabled());

    // PID_Config_Data update_kd; update_kd.set_kd(1);
    // TEST_ASSERT_TRUE(default_pid_handler.applyConfigUpdate(update_kd));
    assignment_test_(PID_Config_Data, kd, 1, applyConfigUpdate);
    assignment_test_(PID_Config_Data, ki, 1, applyConfigUpdate);
    assignment_test_(PID_Config_Data, kp, 1, applyConfigUpdate);
    assignment_test_(PID_Config_Data, motorID, 1, applyConfigUpdate);
    assignment_test_(Set_PID_Setpoint, setPoint, 10, applySetPoint);
    assignment_test_(Set_PID_Control, in_control, true, applyMotorControl);

}
#undef assignment_test_

void PID_update_test(void){
    PIDHandler default_pid_handler;
    default_pid_handler.setEnabled(true);
    PID_Config_Data update_kp; update_kp.set_kp(1);
    default_pid_handler.applyConfigUpdate(update_kp);
    default_pid_handler.set_new_setpoint(100);
    for(size_t a = 0; a < 20; a++){
        TEST_ASSERT_TRUE_MESSAGE(default_pid_handler.update_pid(a), "UPDATE PID");
        Serial.println(default_pid_handler.get_motor_value());
        Wait(1000);
    }
    Serial.println(default_pid_handler.get_motor_value());
    TEST_ASSERT_TRUE_MESSAGE(default_pid_handler.get_motor_value() > 0, "POSITIVE PID OUTPUT");
    default_pid_handler.set_new_setpoint(-100);
    default_pid_handler.update_pid(0);
    TEST_ASSERT_TRUE_MESSAGE(default_pid_handler.get_motor_value() < 0, "NEGATIVE PID OUTPUT");
}