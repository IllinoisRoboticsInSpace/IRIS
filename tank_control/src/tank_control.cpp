/**
 * @file tank_control.cpp
 * @author Rama Rao Vencharla
 * 
 * @brief Tank Control Class to Control SparkMax Motors
 */


#include "tank_control.hpp"

TankControl::TankControl(SparkMax* leftMotor, SparkMax* rightMotor) {
    this->leftTankMotor = leftMotor;
    this->rightTankMotor = rightMotor;
    setupMotors(this->leftTankMotor);
    setupMotors(this->rightTankMotor);
}

void TankControl::setupMotors(SparkMax * motor) {
    motor->SetIdleMode(IdleMode::kCoast);
    motor->SetMotorType(MotorType::kBrushless);
    motor->SetInverted(true);
    motor->BurnFlash();
}

void TankControl::setMotors(double leftMotorDutyCycle, double rightMotorDutyCycle) {
    SparkMax::Heartbeat();
    leftTankMotor->SetDutyCycle(leftMotorDutyCycle);
    rightTankMotor->SetDutyCycle(rightMotorDutyCycle);
}

void TankControl::drive(double leftSpeed, double rightSpeed) {
    // double leftSpeed = throttle + turn;
    // double rightSpeed = throttle - turn;
    if (leftSpeed > 1) { leftSpeed = 1; }
    if (leftSpeed < -1) { leftSpeed = -1; }
    if (rightSpeed > 1) { rightSpeed = 1; }
    if (rightSpeed < -1) { rightSpeed = -1; }
    setMotors(leftSpeed, rightSpeed);

}

void TankControl::straight(double speedPercentage) {
    std::cout << "Going Straight" << std::endl;
    drive(speedPercentage, 0);
}

void TankControl::turn(double angle, double speedPercentage) {
    if (angle > 0) { //pivot turn right
        setMotors(speedPercentage, -speedPercentage); // left forward, right backward
    } else { // pivot turn left
        setMotors(-speedPercentage, speedPercentage); // left backward right forward
    }
}

void TankControl::stop() {
    std::cout << "Stopping" << std::endl;
    setMotors(0, 0);
}


// int main()
// {
//     try
//     {
//         // Initialize SparkMax object with CAN interface and CAN ID
//         SparkMax leftMotor("can0", 1);
//         SparkMax rightMotor("can0", 2);
//         TankControl controller(&leftMotor, &rightMotor);
//         std::cout << "initializing robot" << std::endl;

//         // Enable and run motor
//         controller.setMotors(0.1, 0.1, 5);
//         controller.turn(90, 0.1);
        
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "Error: " << e.what() << std::endl;
//         return -1;
//     }

//     return 0;
// }