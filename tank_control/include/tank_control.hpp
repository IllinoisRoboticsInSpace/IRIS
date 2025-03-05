/**
 * @file tank_control.cpp
 * @author Rushil Shah
 * 
 * @brief Header file for Tank Control Class
 */

#ifndef TANK_CONTROL_HPP
#define TANK_CONTROL_HPP

#include <iostream>
#include "SparkMax.hpp"
#include <string>
#include <unistd.h>

class TankControl {
    public:
        TankControl(SparkMax* leftMotor, SparkMax* rightMotor);

        void setMotors(double leftMotorDutyCycle, double rightMotorDutyCycle, int duration);
        void drive(double throttle, double turn, double durationSeconds);
        void straight(double durationSeconds, double speedPercentage);
        void turn(double angle, double speedPercentage);
        void stop();

    private:
        SparkMax* leftTankMotor;
        SparkMax* rightTankMotor;
};

#endif