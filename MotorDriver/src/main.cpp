#include "Sabertooth.h"
#include "MotorDriver.h"
#include "RotaryEncoder.h"
#include "PID_v1.h"
// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

#define TICKS_TO_DEG 0.3515625 // 360/1024 
#define ENCODER_PIN_IN A2
#define ENCODER_PIN_OUT A3

#define MOTOR_BUTTON_PIN A0

double encoder_read = 0;
double prev_encoder_read = 0;
long encoder_read_long = 0; 
// double temp = 0;
double motor_write = 0;
double setpoint = 1000;
double Kp = 0.003; double Ki = 0.0005; double Kd = 0;
Sabertooth sabertooth(128);
RotaryEncoder encoder(ENCODER_PIN_IN, ENCODER_PIN_OUT, RotaryEncoder::LatchMode::TWO03);

uint32_t last_run = 0;
PID pid(&encoder_read, &motor_write, &setpoint, Kp, Ki, Kd, DIRECT);



void encoder_interupt();

void setup() {

  pinMode(MOTOR_BUTTON_PIN, INPUT);

  pid.SetOutputLimits(-50,50);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(50);
  SabertoothTXPinSerial.begin(9600);
  sabertooth.autobaud();

  Serial.begin(112500);
  Serial.println("Encoder and PID Test");

  sabertooth.motor(0);
  //sabertooth.motor(15);

  attachInterrupt(ENCODER_PIN_IN, encoder_interupt, CHANGE);  
  attachInterrupt(ENCODER_PIN_OUT, encoder_interupt, CHANGE);
}

void encoder_interupt(){
  encoder.tick();
}

void loop() {
  
  encoder_read_long = encoder.getPosition(); //* TICKS_TO_DEG;
  encoder_read = (double)encoder_read_long;
  //Serial.println(digitalRead(MOTOR_BUTTON_PIN));
  pid.Compute();
  if(digitalRead(MOTOR_BUTTON_PIN) == 1){
    //sabertooth.motor(15);
    sabertooth.motor((int)motor_write);
  }
  else{
    sabertooth.motor(0);
  }
  
  
  Serial.print(encoder_read_long); Serial.print(" "); Serial.println((int)motor_write);

  // if(millis() > 3000){
  //   sabertooth.motor(0);
  // }
}