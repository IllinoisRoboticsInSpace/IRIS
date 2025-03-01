#include <iostream>
#include "SparkMax.hpp"
#include <string>
#include <unistd.h>


class TankControl {
    public:
        TankControl(SparkMax* leftMotor, SparkMax* rightMotor) {
            this->leftTankMotor = leftMotor;
            this->rightTankMotor = rightMotor;
            leftTankMotor->SetIdleMode(IdleMode::kCoast);
            leftTankMotor->SetMotorType(MotorType::kBrushless);
            leftTankMotor->SetInverted(true);
            leftTankMotor->BurnFlash();
            rightTankMotor->SetIdleMode(IdleMode::kCoast);
            rightTankMotor->SetMotorType(MotorType::kBrushless);
            rightTankMotor->SetInverted(true);
            rightTankMotor->BurnFlash();
        }
        void setMotors(double leftMotorDutyCycle, double rightMotorDutyCycle, int duration) {
            auto start = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count() < duration)
            {  
                SparkMax::Heartbeat();
                leftTankMotor->SetDutyCycle(leftMotorDutyCycle);
                rightTankMotor->SetDutyCycle(rightMotorDutyCycle);
            }
        }
        void drive(double throttle, double turn) {
            double leftSpeed = throttle + turn;
            double rightSpeed = throttle - turn;
            setMotors(leftSpeed, rightSpeed, 10);
        }
        void straight(double durationSeconds, double speedPercentage) {
            std::cout << "Going Straight" << std::endl;
            drive(speedPercentage, 0);
        }
        void turn(double angle, double speedPercentage) {
            if (angle > 0) { //pivot turn right
                setMotors(speedPercentage, -speedPercentage, 5); //  forward, right backward
            } else { // pivot turn left
                setMotors(-speedPercentage, speedPercentage, 5); // t backward right forward
            }
        }
        void stop() {
            std::cout << "Stopping" << std::endl;
            setMotors(0,0, 10);
        }
    private:
        SparkMax * leftTankMotor;
        SparkMax * rightTankMotor;
};

int main()
{
    try
    {
        // Initialize SparkMax object with CAN interface and CAN ID
        SparkMax leftMotor("can0", 1);
        SparkMax rightMotor("can0", 2);
        TankControl controller(&leftMotor, &rightMotor);
        std::cout << "initializing robot" << std::endl;

        // Enable and run motor
        controller.setMotors(0.1, 0.1, 5);
        controller.turn(90, 0.1);
        
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}