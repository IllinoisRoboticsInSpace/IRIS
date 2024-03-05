#include "Sabertooth.h"
#include "MotorDriver.h"
#include "DebugTools.h"
#include "generated/commands.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

#define DEFAULT_SABERTOOTH_BAUD_RATE 9600 // 9600 is the default baud rate for Sabertooth packet serial.

MotorDriver driver;
// Serial: line from us to arduino
// Serial1: line from arduino to sabertooth

void setup() {
  Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);

  driver = MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE);
  if (driver.initMotorDriver() == false)
  {
    DEBUG_PRINTLN("Initialization Failed");
  }
    PID_Config_Data pid_config;
    pid_config.set_PID_ID(0);
    pid_config.set_enabled(true);
    
    PID_Config_Data pid_config_kp;
    pid_config.set_PID_ID(0);
    pid_config.set_kp(1);

    PID_Config_Data pid_config_encoder_id;
    pid_config.set_PID_ID(0);
    pid_config.set_encoderID(0);

    Encoder_Config_Data encoder_config;
    encoder_config.set_encoderID(0);
    encoder_config.set_enabled(true);

    Set_PID_Setpoint pid_setpoint;
    pid_setpoint.set_PID_ID(0);
    pid_setpoint.set_setPoint(10.3);

    Serial_Message_To_Arduino message;
    message.set_opcode(Opcode_To_Arduino::CONFIG_PID);
    
    message.set_pidConfigData(pid_config);
    driver.execute(message);

    message.set_pidConfigData(pid_config_kp);
    driver.execute(message);

    message.set_pidConfigData(pid_config_encoder_id);
    driver.execute(message);

    message.clear_pidConfigData();
    message.set_opcode(Opcode_To_Arduino::SET_PID_SETPOINT);
    message.set_setPIDSetpoint(pid_setpoint);
    driver.execute(message);

    message.clear_setPIDControl();
    message.set_opcode(Opcode_To_Arduino::CONFIG_ENCODER);
    message.set_encoderConfigData(encoder_config);

}

void loop() {
    // driver.getEncoderConfig(0).set_encoder_tick_count(driver.getEncoderConfig(0).get_encoder_tick_count() + driver.getPIDConfig(0).get_motor_value());
    driver.set_encoder_ticks(0, driver.get_encoder_ticks(0) + driver.get_pid_output(0));
    
    Serial.print("encoder ticks: "); Serial.println(driver.get_encoder_ticks(0));
    Serial.print("pid output: "); Serial.println(driver.get_pid_output(0));
    driver.update();
}