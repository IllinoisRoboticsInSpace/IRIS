#include "Sabertooth.h"
#include "MotorDriver.h"
#include "RotaryEncoder.h"
#include "PID_v1.h"
// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

#define TICKS_TO_DEG 0.3515625 // 360/1024 
//#define SABERTOOTH_PIN A5
#define ENCODER_PIN_IN 3
#define ENCODER_PIN_OUT 2

double encoder_read = 0;
double prev_encoder_read = 0;
// double temp = 0;
double motor_write = 0;
double setpoint = 360;
double Kp = 1; double Ki = 0.1; double Kd = 0.01;
Sabertooth sabertooth(128);
RotaryEncoder encoder(ENCODER_PIN_IN, ENCODER_PIN_OUT, RotaryEncoder::LatchMode::TWO03);

uint32_t last_run = 0;
PID pid(&encoder_read, &motor_write, &setpoint, Kp, Ki, Kd, DIRECT);

//void encoder_interupt();

void setup() {
  pid.SetOutputLimits(-127,127);
  
  SabertoothTXPinSerial.begin(9600);
  sabertooth.autobaud();

  Serial.begin(112500);
  Serial.println("Encoder and PID Test");

  //sabertooth.motor(0);
  sabertooth.motor(15);
  //attachInterrupt(ENCODER_PIN_IN, encoder_interupt, FALLING);
  //analogWrite(13, LOW);
}

// void encoder_interupt(){
//   if(millis() - last_run >= 100){
//     encoder.tick();
//     encoder_read = encoder.getPosition();
//     Serial.println(encoder_read);
//     last_run = millis();
//   }
// }

void loop() {
  encoder.tick();
  //prev_encoder_read = encoder_read;
  //sabertooth.motor(10);
  //encoder.setPosition(encoder.getPosition() + 1);
  
  encoder_read = encoder.getPosition(); //* TICKS_TO_DEG;
  //pid.Compute();
  //sabertooth.motor((int)motor_write);
  // if(prev_encoder_read != encoder_read){
  //   Serial.println(encoder_read);
  // }
  Serial.print(encoder_read); //Serial.print(","); Serial.print(encoder.getDirection());
  Serial.print(" ");
  Serial.println((int)encoder.getDirection());
  // if(abs(encoder_read) > 120){
  //   analogWrite(13, HIGH);
  // }

  if(millis() > 10000){
    sabertooth.motor(0);
  }
  // if(encoder_read > 100 || encoder_read < -100){
  //   sabertooth.motor(0);
  // }
  //encoder.tick();
  //encoder.tick();

  //encoder_read = encoder.getPosition();
  
}