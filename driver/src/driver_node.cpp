#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "SparkFlex.hpp"

using namespace std::chrono_literals;

class Driver : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tread_speed_left_teleop_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tread_speed_right_teleop_subscription;
    
    // FIX 1: Use TimerBase instead of templated WallTimer
    rclcpp::TimerBase::SharedPtr timer;

    SparkFlex left;
    SparkFlex right;
    float incoming_left;
    float incoming_right;
public:
    Driver();
    auto handle_incoming_left(float in) -> void;
    auto handle_incoming_right(float in) -> void;
    auto periodic() -> void;
};

Driver::Driver() : Node("Driver"), left("can0", 1), right("can0", 2), incoming_left(0), incoming_right(0) {
    tread_speed_left_teleop_subscription  = this->create_subscription<std_msgs::msg::Float32>(
        "tread_speed_left_teleop", 10,  
        [this](const std_msgs::msg::Float32::SharedPtr msg) { handle_incoming_left(msg->data); }
    );
    
    tread_speed_right_teleop_subscription = this->create_subscription<std_msgs::msg::Float32>(
        "tread_speed_right_teleop", 10, 
        [this](const std_msgs::msg::Float32::SharedPtr msg) { handle_incoming_right(msg->data); }
    );
    
    timer = this->create_wall_timer(10ms, std::bind(&Driver::periodic, this));

    left.SetIdleMode(IdleMode::kCoast);
    left.SetMotorType(MotorType::kBrushless);
    left.SetInverted(false);
    left.BurnFlash();

    right.SetIdleMode(IdleMode::kCoast);
    right.SetMotorType(MotorType::kBrushless);
    right.SetInverted(true);
    right.BurnFlash();
}

auto Driver::handle_incoming_left(float in) -> void {
    std::cout << "[Driver::handle_incoming_left] " << in << std::endl;
    incoming_left = in;
}

auto Driver::handle_incoming_right(float in) -> void {
    std::cout << "[Driver::handle_incoming_right] " << in << std::endl;
    incoming_right = in;
}

auto Driver::periodic() -> void {
    left.SetVoltage(incoming_left * 12.0);
    right.SetVoltage(incoming_right * 12.0);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Driver>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}