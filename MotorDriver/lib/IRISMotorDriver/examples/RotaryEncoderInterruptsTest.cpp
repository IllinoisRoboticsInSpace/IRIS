#include "Sabertooth.h"
#include "MotorDriver.h"
// #include "DebugTools.h"
#include "generated/commands.h"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

// Serial: line from us to arduino
#define PIN_IN1 50
#define PIN_IN2 51

long counter = 0;
long counter2 = 0;

class Dummy_Encoder
{
  public:
    Dummy_Encoder();
    long get_encoder_tick_count()
    {
        return encoder->getPosition();
    }
    void pin1InterruptHandler()
    {
        encoder->tick();
        // encoder->setPosition(millis());
        // counter = millis();
    }

    void pin2InterruptHandler()
    {
        encoder->tick();
        // encoder->setPosition(millis());
        // counter2 = millis();/
    }

  private:
    RotaryEncoder* encoder;
    bindArgVoidFunc_t interruptGate_pin1 = nullptr;
    bindArgVoidFunc_t interruptGate_pin2 = nullptr;
};

Dummy_Encoder::Dummy_Encoder()
{
  // pinMode(PIN_IN1, INPUT);
  // pinMode(PIN_IN2, INPUT);

  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  interruptGate_pin1 = bindArgGateThisAllocate(&Dummy_Encoder::pin1InterruptHandler, this);
  interruptGate_pin2 = bindArgGateThisAllocate(&Dummy_Encoder::pin2InterruptHandler, this);

  attachInterrupt(digitalPinToInterrupt(PIN_IN1), interruptGate_pin1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), interruptGate_pin2, CHANGE);

  // Serial.println("HERE");
  
  // attachInterrupt(digitalPinToInterrupt(PIN_IN1), (void (*)()) &(this->pin1InterruptHandler), CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_IN2), interruptGate_pin2, CHANGE);
}

// RotaryEncoder *encoder = nullptr;

// void checkPosition()
// {
//   encoder->tick(); // just call tick() to check the state.
// }

Dummy_Encoder encoder;

void setup() {
  Serial.begin(DEFAULT_HOST_SERIAL_BAUD_RATE);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  while (!Serial){}
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("HERE2");



  // pinMode(PIN_IN1, INPUT_PULLUP);
  // pinMode(PIN_IN2, INPUT_PULLUP);

  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  // attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  // encoder = Dummy_Encoder();
}

void loop() {

  static int pos = 0;

  // encoder->tick(); // just call tick() to check the state.

  int newPos = encoder.get_encoder_tick_count();
  if (pos != newPos) {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print("\n");
    // Serial.print(" dir:");
    // Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  } // if

  // DEBUG_PRINTF("counter 1: %ld\n", counter);
  // DEBUG_PRINTF("counter 2: %ld\n", counter2);

  // DEBUG_PRINTF("1:%ld\t2:%ld\t3:%ld\t4:%ld\n", driver.encoder_configs[0].get_encoder_tick_count() 
  //                                            , driver.encoder_configs[1].get_encoder_tick_count()
  //                                            , driver.encoder_configs[2].get_encoder_tick_count()
  //                                            , driver.encoder_configs[3].get_encoder_tick_count());

  // DEBUG_PRINTF("1:%ld\t2:%ld\t3:%ld\t4:%ld\n", driver.encoder_configs[0].encoder->getPosition() 
  //                                            , driver.encoder_configs[1].encoder->getPosition()
  //                                            , driver.encoder_configs[2].encoder->getPosition()
  //                                            , driver.encoder_configs[3].encoder->getPosition());
}