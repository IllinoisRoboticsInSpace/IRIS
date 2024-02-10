#include "Sabertooth.h"
#include "MotorDriver.h"
#include "DebugTools.h"
#include "generated/commands.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>
#include <RotaryEncoder.h>
#define PIN_IN1_L 50
#define PIN_IN2_L 52
#define PIN_IN1_R 30
#define PIN_IN2_R 32

RotaryEncoder *encoder_L = nullptr;
RotaryEncoder *encoder_R = nullptr;

#define DEFAULT_SABERTOOTH_BAUD_RATE 9600 // 9600 is the default baud rate for Sabertooth packet serial.

// MotorDriver driver;
int sab_address = 130;
Sabertooth ST(sab_address);

void checkPosition_L()
{
  encoder_L->tick(); // just call tick() to check the state.
}

void checkPosition_R()
{
  encoder_R->tick(); // just call tick() to check the state.
}

// Serial: line from us to arduino
// Serial1: line from arduino to sabertooth

void setup() {
  Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
  Serial1.begin(DEFAULT_SABERTOOTH_BAUD_RATE);


  // driver = MotorDriver(DEFAULT_HOST_SERIAL_BAUD_RATE);

  // if (driver.initMotorDriver() == false)
  // {
  //   DEBUG_PRINTLN("Initialization Failed");
  // }
  DEBUG_PRINTF("Setup Sabertooth: Address %d\n", sab_address);

  encoder_L = new RotaryEncoder(PIN_IN1_L, PIN_IN2_L, RotaryEncoder::LatchMode::TWO03);
  encoder_R = new RotaryEncoder(PIN_IN1_R, PIN_IN2_R, RotaryEncoder::LatchMode::TWO03);

  pinMode(PIN_IN1_L, INPUT_PULLUP);
  pinMode(PIN_IN2_L, INPUT_PULLUP);
  pinMode(PIN_IN1_R, INPUT_PULLUP);
  pinMode(PIN_IN2_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_IN1_L), checkPosition_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2_L), checkPosition_L, CHANGE);

  attachInterrupt(digitalPinToInterrupt(PIN_IN1_R), checkPosition_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2_R), checkPosition_R, CHANGE);

}

void loop() {
  // driver.update();

  static int pos_L = 0;
  static int pos_R = 0;
  int newPos_L, newPos_R;

  int max = 30;
  int pause = 5 * 1000;
  int stop_delay = 3 * 1000;

  encoder_L->tick(); // just call tick() to check the state.
  encoder_R->tick();

  newPos_L = encoder_L->getPosition();
  // if (pos_L != newPos_L) {
    DEBUG_PRINTF("LEFT, pos:%d, dir:%d\n", newPos_L, encoder_L->getDirection());
    pos_L = newPos_L;
  // }

  newPos_R = encoder_R->getPosition();
  // if (pos_R != newPos_R) {
    DEBUG_PRINTF("RIGHT, pos:%d, dir:%d\n", newPos_R, encoder_R->getDirection());
    pos_R = newPos_R;
  // }

  ST.motor(1, max);  // Go forward at full power.
  ST.motor(2, max);
  DEBUG_PRINTF("Power set to 1: %d, 2: %d\n", max, max);
  delay(pause);       // Wait 2 seconds.
  ST.motor(1, 0);    // Stop.
  ST.motor(2, 0);
  DEBUG_PRINTF("Power set to 1: %d, 2: %d\n", 0, 0);

  encoder_L->tick(); // just call tick() to check the state.
  encoder_R->tick();

  newPos_L = encoder_L->getPosition();
  // if (pos_L != newPos_L) {
    DEBUG_PRINTF("LEFT, pos:%d, dir:%d\n", newPos_L, encoder_L->getDirection());
    pos_L = newPos_L;
  // }

  newPos_R = encoder_R->getPosition();
  // if (pos_R != newPos_R) {
    DEBUG_PRINTF("RIGHT, pos:%d, dir:%d\n", newPos_R, encoder_R->getDirection());
    pos_R = newPos_R;
  // }

  delay(stop_delay);       // Wait 2 seconds.
  ST.motor(1, -max); // Reverse at full power.
  ST.motor(2, -max);
  DEBUG_PRINTF("Power set to 1: %d, 2: %d\n", -max, -max);
  delay(pause);       // Wait 2 seconds.
  ST.motor(1, 0);    // Stop.
  ST.motor(2, 0);
  DEBUG_PRINTF("Power set to 1: %d, 2: %d\n", 0, 0);
  delay(stop_delay);
}