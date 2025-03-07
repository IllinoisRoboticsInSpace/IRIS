/**
 * @file joy_to_tank_control.hpp
 * @author Rushil Shah
 * 
 * @brief Header File for gamepad_to_motor Node
 */

#ifndef JOY_TO_TANK_CONTROL_HPP
#define JOY_TO_TANK_CONTROL_HPP

#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tank_control.hpp"
#include "SparkMax.hpp"

using std::placeholders::_1;

const float DEADZONE = 0.1;

class Gamepad_to_Motor : public rclcpp::Node {
    public:

        Gamepad_to_Motor();
    
    private:

        // Callbacks
        void left_joystick_callback(const std_msgs::msg::Float32& msg); /* Basic Driving - Tank Control */ /* TODO */
        void right_joystick_callback(const std_msgs::msg::Float32& msg); /* Basic Driving - Tank Control */ /* TODO */
        void point_turn_left_callback(const std_msgs::msg::Bool& msg); /* Precision Point-Turning Left */ /* TODO */
        void point_turn_right_callback(const std_msgs::msg::Bool& msg); /* Precision Point-Turning Right */ /* TODO */
        void hard_stop_callback(const std_msgs::msg::Bool& msg); /* Stop all Motor Current */ /* TODO */
        
        double joystick_to_throttle(float left, float right); /* Convert Joystick Input to Throttle */
        double joystick_to_turn(float left, float right);

        // Caches
        float last_command_left_joy;
        float last_command_right_joy;
        bool killed;

        // Tank Control Objects
        SparkMax leftMotor;
        SparkMax rightMotor;
        TankControl tank_controller;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_drive_sub; /* /gamepad/left_drive */
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_drive_sub; /* /gamepad/right_drive */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_lb_sub; /* /gamepad/left_back_cltn_mtr */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_rb_sub; /* /gamepad/right_back_cltn_mtr */
        // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_a_sub; /* /gamepad/exc_intrnl_mtr */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_b_sub; /* /gamepad/exc_thrd_rod_actr */
};

#endif