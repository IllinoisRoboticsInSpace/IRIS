/**
 * @file cytron_linear_actuator.hpp
 * @author Rama Rao Vencharla
 * 
 * @brief Header File for Linear Actuator Node
 */

#ifndef CYTRON_LINEAR_ACTUATOR_HPP
#define CYTRON_LINEAR_ACTUATOR_HPP

#include <CytronMotorDriver.h>
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

class CytronLinearActuator : public rclcpp::Node {
    public:

        CytronLinearActuator();

    private:

        // Callbacks
        void linear_actuator_callback(const std_msgs::msg::Float32& msg);
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lin_actr; /* /gamepad/lin_actr */
};

#endif // CYTRON_LINEAR_ACTUATOR_HPP