#include <mutex>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "SparkMax.hpp"
#include "constants.hpp"

#define N_INPUTS = 6

std::string[] keys = ["left_drive", "right_drive", 
                      "excavator_lin_actr",
                      "auto_mode", "stop_mode", "speed_mode"];

class JetsonSparkMax : public rclcpp::Node {

    public:
        JetsonSparkMax()
            : Node("jetson_sparkmax"),
            m_left_drive("can0", constants::LEFT_DRIVE),
            m_right_drive("can0", constants::RIGHT_DRIVE),
            m_left_back_coll("can0", constants::LEFT_BACK_COLL),
            m_right_back_coll("can0", constants::RIGHT_BACK_COLL),
            m_exc_internal("can0", constants::EXC_INTERNAL),
            m_exc_thread_rod("can0", constants::EXC_THREAD_ROD),
            m_exc_pivot_lin("can0", constants::EXC_PIVOT_LIN) {
            logger_ = this->get_logger();

            // Initialize gamepad subscriptions
            for (const auto& mapping : get_gamepad_mapping()) {
                const std::string& key = mapping.first;
                int value = mapping.second;
                rclcpp::MessageType msg_type = (key == "left_drive" || key == "right_drive") ?
                    std::make_shared<std_msgs::msg::Float32>() : std::make_shared<std_msgs::msg::Bool>();
                auto callback = std::bind(&JetsonCommsNode::motor_driver_callback, this, _1, key);
                gamepad_subscribers[value] = create_subscription<decltype(msg_type)>(
                    f"/gamepad/{key}", 10, callback);
                RCLCPP_INFO(logger_, "Created node %s", this->get_name());
            }
        }
    private:
        std::shared_ptr<std::mutex> lock_;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> gamepad_subscribers_;
        MotorDriver[2] motor_drivers_;
        MotorDriver lin_actuator_driver; /* TODO: Overload MotorDriver Functions for Cytron */
        rclcpp::Logger logger_;
        
        void motor_driver_callback(const std::shared_ptr<void> msg, const std::string& motor) {
            std::lock_guard<std::mutex> lock(lock_);
            if (std::holds_alternative<std_msgs::msg::Float32>(msg)) {
                const auto& float_msg = std::get<std_msgs::msg::Float32>(msg);
                RCLCPP_INFO(logger_, "Data received on %s: %f", motor.c_str(), float_msg.data);
            } else if (std::holds_alternative<std_msgs::msg::Bool>(msg)) {
                const auto& bool_msg = std::get<std_msgs::msg::Bool>(msg);
                RCLCPP_INFO(logger_, "Data received on %s: %s", motor.c_str(), bool_msg.data ? "true" : "false");
            }
            if (motor == "left_drive") {
                m_left_drive.Heartbeat();
            } else if (motor == "right_drive") {
                m_right_drive.Heartbeat();
            } else if (motor == "left_back_cltn_mtr") {
                m_left_back_coll.Heartbeat();
            } else if (motor == "right_back_cltn_mtr") {
                m_right_back_coll.Heartbeat();
            } else if (motor == "exc_intrnl_mtr") {
                m_exc_internal.Heartbeat();
            } else if (motor == "exc_thrd_rod_actr") {
                m_exc_thread_rod.Heartbeat();
            } else if (motor == "exc_pvt_lin_actr") {
                m_exc_pivot_lin.Heartbeat();
            } else if (motor == "auto_mode") {
                switch_to_auto_mode();
            } else if (motor == "stop_mode") {
                stop_motors();
            }
        }
        void switch_to_auto_mode() {
            // Not implemented in original
        }
        void stop_motors() {
            m_left_drive.SetDutyCycle(0);
            m_right_drive.SetDutyCycle(0);
            m_left_back_coll.SetDutyCycle(0);
            m_right_back_coll.SetDutyCycle(0);
            m_exc_internal.SetDutyCycle(0);
            m_exc_pivot_lin.SetDutyCycle(0);
            m_exc_thread_rod.SetDutyCycle(0);
        }
};
// Entry function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetsonCommsNode>());
    rclcpp::shutdown();
    return 0;
}