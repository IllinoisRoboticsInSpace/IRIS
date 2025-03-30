#include "cytron_linear_actuator.hpp"

using namespace std::placeholders;

CytronLinearActuator::CytronLinearActuator() : Node("cytron_linear_actuator") {
    try {
        // Initialize the motor driver
        this->motor = CytronMD(PWM_DIR, 3, 5); // 3 -> PWM and 5 -> DIR pins for the motor driver
        // Create a subscriber to listen to the gamepad topic
        this->lin_actr = this->create_subscription<std_msgs::msg::Float32>(
            "/gamepad/lin_actr", 10, std::bind(&CytronLinearActuator::linear_actuator_callback, this, std::placeholders::_1)
        );

        if (!this->lin_actr) {
            RCLCPP_ERROR(this->get_logger(), "Error creating subscriber for /gamepad/lin_actr");
            throw std::runtime_error("Error creating subscriber for /gamepad/lin_actr");
        }
        RCLCPP_INFO(this->get_logger(), "Cytron Linear Actuator Node Initialized");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing linear actuator motor driver: %s", e.what());
        throw std::runtime_error("Error initializing linear actuator motor driver");
    }

    // Create a timer to periodically check the motor state
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CytronLinearActuator::timer_callback, this)
    );  
} 

void CytronLinearActuator::linear_actuator_callback(const std_msgs::msg::Float32& msg) {
    float speed = msg.data;
    // Check if the speed is within the valid range
    if (speed < -255 || speed > 255) {
        RCLCPP_WARN(this->get_logger(), "Speed out of range. Must be between -255 and 255.");
        return;
    }
    // Set the speed of the motor
    this->motor.setSpeed(speed);
    RCLCPP_INFO(this->get_logger(), "Setting motor speed to %f", speed);
}

void CytronLinearActuator::timer_callback() {
    // Periodically check the motor state or perform any other tasks
    RCLCPP_INFO(this->get_logger(), "Current motor speed: %d", this->motor.getSpeed());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CytronLinearActuator>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}