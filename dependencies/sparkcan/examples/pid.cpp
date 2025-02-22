#include <iostream>
#include <iomanip>
#include <chrono>
#include "SparkMax.hpp"
#include "PIDController.hpp"

/*
*This feature is in development*
This is currently being tested with a SPARK MAX while connected to a NEO 550 Brushless Motor.
*/

int main()
{
    try
    {
        // Initialize SparkMax object with CAN interface and CAN ID
        SparkMax motor("can0", 47);
        motor.SetIdleMode(IdleMode::kBrake);
        motor.SetMotorType(MotorType::kBrushless);
        
        // Obtain a PIDController object from the motor
        PIDController pidController(motor);

        // Configure PID settings
        uint8_t slot = 0;  // Use slot 0 for the PID configuration
        pidController.SetP(slot, 0.00006f);        // Set proportional gain
        pidController.SetI(slot, 0.000001f);      // Set integral gain
        pidController.SetD(slot, 1.0f);       // Set derivative gain
        pidController.SetIZone(slot, 0.0f);
        pidController.SetF(slot, 0.000015f);        // Set feedforward gain
        //pidController.SetOutputMin(slot, -1.0f); 
        //pidController.SetOutputMax(slot, 1.0f);

        // Configure smart motion settings for velocity control
        pidController.SetSmartMotionMaxVelocity(slot, 5000.0f);  // Max velocity in RPM
        pidController.SetSmartMotionMaxAccel(slot, 3000.0f);     // Max acceleration in RPM/s

        // Burn PID settings
        motor.BurnFlash();

        std::cout << "PID Configuration applied successfully." << std::endl;

        // Loop for 10 seconds
        auto start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count() < 10)
        {
            // Set velocity setpoint
            float targetVelocity = 3000.0f;  // Target velocity in RPM
            pidController.SetReference(targetVelocity, CtrlType::kVelocity);

            // Print the current velocity setpoint
            std::cout << "Target Velocity: " << targetVelocity << " RPM" << std::endl;

            // Add a short delay between loops
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop the motor after the loop
        motor.SetSetpoint(0.0f);
        std::cout << "Motor stopped." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
