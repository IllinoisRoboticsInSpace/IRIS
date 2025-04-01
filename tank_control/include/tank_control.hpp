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
#include <chrono>
#include <cmath>

class TankControl {
    public:
        TankControl(SparkMax* leftMotor, SparkMax* rightMotor);

        void setMotors(double leftMotorDutyCycle, double rightMotorDutyCycle);
        void drive(double throttle, double turn);
        void straight(double speedPercentage);
        void turn(double angle, double speedPercentage);
        void stop();
        void setupMotors(SparkMax * motor);
        void getStatus();

    private:
        SparkMax* leftTankMotor;
        SparkMax* rightTankMotor;
};

#endif