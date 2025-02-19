#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <thread>
#include <mutex>

#define N_INPUTS = 6

std::string[] keys = ["left_drive", "right_drive", 
                      "excavator_lin_actr",
                      "auto_mode", "stop_mode", "speed_mode"]

class SparkMaxCommunication_Sub : public rclcpp::Node {
    public:
        SparkMaxCommunication_Sub : Node("SparkMaxCommunication_Sub") {
            lock_ = std::make_shared<std::mutex>();
            gamepad_subscribers_.resize(N_INPUTS);
            
            // Initialize Subscribers
            for (const std::string key : keys) {
                if (key.equals("left_drive") || key == "right_drive") {
                    gamepad_subscribers_[]
                }
            }
        }

    private: 
        std::shared_ptr<std::mutex> lock_;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> gamepad_subscribers_;
        MotorDriver[2] motor_drivers_;
        MotorDriver lin_actuator_driver; /* TODO: Overload MotorDriver Functions for Cytron */
}