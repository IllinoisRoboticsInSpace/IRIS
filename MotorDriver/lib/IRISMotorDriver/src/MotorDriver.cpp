#include "MotorDriver.h"

void init_motor_driver()
{
    Serial.begin(112500);
    Serial.println("Motor Driver Initialized");
}