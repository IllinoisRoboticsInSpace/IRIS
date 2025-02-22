#include <iostream>
#include "SparkMax.hpp"
#include "SparkFlex.hpp"

/*
This has been tested with the SPARK MAX while connected to an AndyMark 775 RedLine Motor and
with a Spark Flex connected to a NEO Vortex Brushless Motor.
*/

int main()
{
    try
    {
        // Initialize SparkMax object with CAN interface and CAN ID
        SparkMax motor("can0", 47);
        SparkFlex motor2("can0", 15);

        // Configure and burn parmaters for Redline
        motor.SetIdleMode(IdleMode::kBrake);
        motor.SetMotorType(MotorType::kBrushed);
        motor.SetInverted(true);
        motor.BurnFlash();

        // Configure and burn parmaters for Neo Vortex
        motor2.SetIdleMode(IdleMode::kBrake);
        motor2.SetMotorType(MotorType::kBrushless);
        motor2.SetSensorType(SensorType::kHallSensor);
        motor2.SetRampRate(0.1);
        motor2.SetInverted(false);
        motor2.SetMotorKv(565);
        motor2.SetEncoderCountsPerRev(7168);
        motor2.SetSmartCurrentFreeLimit(20.0);
        motor2.SetSmartCurrentStallLimit(20.0);
        motor2.BurnFlash();
   
        // Loop for 10 seconds
        auto start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count() < 10)
        {
            // Enable and run motors
            motor.Heartbeat();
            motor2.Heartbeat();

            motor.SetDutyCycle(0.05);
            motor2.SetDutyCycle(0.1);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
