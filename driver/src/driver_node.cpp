#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "SparkMax.hpp"

using namespace std::chrono_literals;

class Driver : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tread_speed_left_teleop_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tread_speed_right_teleop_subscription;
    
    // FIX 1: Use TimerBase instead of templated WallTimer
    rclcpp::TimerBase::SharedPtr timer;

    SparkMax left;
    SparkMax right;
public:
    Driver();
    auto handle_incoming_left(float in) -> void;
    auto handle_incoming_right(float in) -> void;
    auto periodic() -> void;
};

Driver::Driver() : Node("Driver"), left("LEFT", 0), right("RIGHT", 1) {
    tread_speed_left_teleop_subscription  = this->create_subscription<std_msgs::msg::Float32>(
        "tread_speed_left_teleop", 10,  
        [this](const std_msgs::msg::Float32::SharedPtr msg) { handle_incoming_left(msg->data); }
    );
    
    tread_speed_right_teleop_subscription = this->create_subscription<std_msgs::msg::Float32>(
        "tread_speed_right_teleop", 10, 
        [this](const std_msgs::msg::Float32::SharedPtr msg) { handle_incoming_right(msg->data); }
    );
    
    timer = this->create_wall_timer(10ms, std::bind(&Driver::periodic, this));
}

auto Driver::handle_incoming_left(float in) -> void {
    left.SetVoltage(in / 12);
}

auto Driver::handle_incoming_right(float in) -> void {
    right.SetVoltage(in / 12);
}

auto Driver::periodic() -> void {
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