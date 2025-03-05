/**
 * @file joy_to_tank_control.hpp
 * @author Rushil Shah
 * 
 * @brief Header File for gamepad_to_motor Node
 */

#ifndef JOY_TO_TANK_CONTROL_HPP
#define JOY_TO_TANK_CONTORL_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class Gamepad_to_Motor : public rclcpp::Node {
    public:

        Gamepad_to_Motor() : Node("gamepad_to_motor");
    
    private:

        void joystick_callback(const std_msgs::msg::Float32& msg) const; /* Basic Driving - Tank Control */ /* TODO */
        void point_turn_left_callback(const std_msgs::msg::Bool& msg) const; /* Precision Point-Turning Left */ /* TODO */
        void point_turn_right_callback(const std_msgs::msg::Bool& msg) const; /* Precision Point-Turning Right */ /* TODO */
        void lin_actr_excavator_callback(const std_msgs::msg::Bool& msg) const; /* Activate Linear Actuator */ /* TODO */
        void hard_stop_callback(const std_msgs::msg::Bool& msg) const; /* Stop all Motor Current */ /* TODO */
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_drive_sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_drive_sub;

};

#endif JOY_TO_TANK_CONTROL_HPP