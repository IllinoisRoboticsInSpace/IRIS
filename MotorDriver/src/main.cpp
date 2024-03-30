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

std::array<SabertoothOperator, MAX_MOTOR_CONFIGS> motor_configs;
std::array<RotaryEncoderOperator, MAX_ENCODER_CONFIGS> encoder_configs;


void setup() {
  Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set configs
  motor_configs[0] = SabertoothOperator(128, DEFAULT_SABERTOOTH_BAUD_RATE, 2, Serial1, false, true);
  motor_configs[1] = SabertoothOperator(128, DEFAULT_SABERTOOTH_BAUD_RATE, 1, Serial1, false, true);
  motor_configs[2] = SabertoothOperator(129, DEFAULT_SABERTOOTH_BAUD_RATE, 1, Serial1, false, true);
  // Needs to be set to cytron implementation
  // motor_configs[3] = SabertoothOperator(130, DEFAULT_SABERTOOTH_BAUD_RATE, 2, Serial1, false, true);

  encoder_configs[0] = RotaryEncoderOperator(40, 41, RotaryEncoder::LatchMode::FOUR3);//Left
  encoder_configs[0].setEnabled(true);
  encoder_configs[1] = RotaryEncoderOperator(36, 37, RotaryEncoder::LatchMode::FOUR3);//Right
  encoder_configs[1].setEnabled(true);
  encoder_configs[2] = RotaryEncoderOperator(44, 45, RotaryEncoder::LatchMode::FOUR3);//Free Big
  encoder_configs[2].setEnabled(true);
  encoder_configs[3] = RotaryEncoderOperator(48, 49, RotaryEncoder::LatchMode::FOUR3);//Excavator
  encoder_configs[3].setEnabled(true);


  driver = MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE, motor_configs);//, encoder_configs);


  if (driver.initMotorDriver() == false)
  {
    DEBUG_PRINTLN("Initialization Failed");
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // driver.update();

  Serial_Message_To_Arduino command;
  command.set_opcode(Opcode_To_Arduino::TURN_MOTOR);
  Turn_Motor turn_motor_command;

  turn_motor_command.set_motorID(0);
  turn_motor_command.set_percentOutput(.50);
  command.set_motorCommand(turn_motor_command);
  driver.execute(command);

  turn_motor_command.set_motorID(2);
  turn_motor_command.set_percentOutput(-.50);
  command.set_motorCommand(turn_motor_command);
  driver.execute(command);
  delay(2000);
  turn_motor_command.set_motorID(0);
  turn_motor_command.set_percentOutput(0);
  command.set_motorCommand(turn_motor_command);
  driver.execute(command);

  turn_motor_command.set_motorID(2);
  turn_motor_command.set_percentOutput(0);
  command.set_motorCommand(turn_motor_command);
  driver.execute(command);
  delay(5000);


  // DEBUG_PRINTF("1:%ld\t2:%ld\t3:%ld\t4:%ld\n", driver.encoder_configs[0].get_encoder_tick_count() 
  //                                            , driver.encoder_configs[1].get_encoder_tick_count()
  //                                            , driver.encoder_configs[2].get_encoder_tick_count()
  //                                            , driver.encoder_configs[3].get_encoder_tick_count());

  // DEBUG_PRINTF("1:%ld\t2:%ld\t3:%ld\t4:%ld\n", driver.encoder_configs[0].encoder->getPosition() 
  //                                            , driver.encoder_configs[1].encoder->getPosition()
  //                                            , driver.encoder_configs[2].encoder->getPosition()
  //                                            , driver.encoder_configs[3].encoder->getPosition());

  // long a = millis();                                     
  // DEBUG_PRINTF("%ld", a);

  // delay(1000);
}