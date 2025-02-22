#include <mutex>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "SparkMax.hpp"
#include "constants.hpp"

#define N_INPUTS 6

constexpr std::array<const char*, 6> KEYS = {
                    "left_drive", 
                    "right_drive", 
                    "excavator_lin_actr",
                    "auto_mode", 
                    "stop_mode", 
                    "speed_mode"
};

class JetsonSparkMax : public rclcpp::Node {

    public:
        JetsonSparkMax()
            : Node("jetson_sparkmax"),
            m_left_drive("can0", constants::LEFT_DRIVE),
            m_right_drive("can0", constants::RIGHT_DRIVE),
            m_excavator_lin_actr("can0", constants::LEFT_BACK_COLL),
            lock_(std::make_shared<std::mutex>()) {

            logger_ = this->get_logger();

            // Initialize gamepad subscriptions
            for (const auto& key : KEYS) {
                
                if (key == "left_drive" || key == "right_drive") {
                    auto subscriber = this->create_subscription<std_msgs::msg::Float32>(
                        std::string("/gamepad/") + key, 
                        10, 
                        std::bind(
                            &JetsonSparkMax::motor_driver_callback_float,
                            this,
                            std::placeholder::_1,
                            key
                        )
                    );
                    gamepad_subscribers_.push_back(subscriber);
                } else {
                    auto subscriber = this->create_subscription<std_msgs::msg::Bool>(
                        std::string("/gamepad/") + key,
                        10,
                        std::bind(
                            &JetsonSparkMax::motor_driver_callback_bool,
                            this, 
                            std::placeholders::_1, 
                            key
                        )
                    );
                    gamepad_subscribers_.push_back(subscriber);
                }
                RCLCPP_INFO(logger_, "Created Subscription %s", this->get_name());
            }
        }
    private:

        std::shared_ptr<std::mutex> lock_;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> gamepad_subscribers_;
        MotorDriver motor_drivers_[2];
        MotorDriver lin_actuator_driver; /* TODO: Overload MotorDriver Functions for Cytron */
        rclcpp::Logger logger_;
        
        void motor_driver_callback_float(
            const std_msgs::msg::Float32::SharedPtr msg, 
            const std::string& motor) {

            std::lock_guard<std::mutex> lock(lock_);

            RCLCPP_INFO(logger_, "Data received on %s: %f", motor.c_str(), msg.data);

            if (motor == "left_drive") {
                m_left_drive.Heartbeat();
            } else if (motor == "right_drive") {
                m_right_drive.Heartbeat();
            } else if (motor == "excavator_lin_actr") {
                //m_excavator_lin_actr.Heartbeat();
            } else if (motor == "auto_mode") {
                switch_to_auto_mode();
            } else if (motor == "stop_mode") {
                stop_motors();
            } else if (motor == "speed_mode") {
                switch_to_speed_mode();
            }
        }

        void motor_driver_callback_bool(
            const std_msgs::msg::Bool::SharedPtr msg, 
            const std::string& motor) {

            std::lock_guard<std::mutex> lock(lock_);

            RCLCPP_INFO(logger_, "Data received on %s: %f", motor.c_str(), msg.data ? "true" : "false");

            if (motor == "left_drive") {
                m_left_drive.Heartbeat();
            } else if (motor == "right_drive") {
                m_right_drive.Heartbeat();
            } else if (motor == "excavator_lin_actr") {
                m_excavator_lin_actr.Heartbeat();
            } else if (motor == "auto_mode") {
                switch_to_auto_mode();
            } else if (motor == "stop_mode") {
                stop_motors();
            } else if (motor == "speed_mode") {
                switch_to_speed_mode();
            }
        }

        void switch_to_auto_mode() {
            // Not implemented in original
        }

        void switch_to_speed_mode() {
            // Not implemented
        }

        void stop_motors() {
            m_left_drive.SetDutyCycle(0);
            m_right_drive.SetDutyCycle(0);
            m_excavator_lin_actr.SetDutyCycle(0);
        }
};

// Entry function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetsonSparkMax>());
    rclcpp::shutdown();
    return 0;
}