/**
 * @file joy_to_tank_control.cpp
 * @author Rushil Shah
 * 
 * @brief Integration b/w Xbox Controller & Motors via Tank Control Class
 */

#include "joy_to_tank_control.hpp"

Gamepad_to_Motor::Gamepad_to_Motor() : Node("gamepad_to_motor"), 
        leftMotor("can0", 1),
        rightMotor("can0", 2),
        tank_controller(&leftMotor, &rightMotor) {

  // Initialize Subscribers
  try {
    this->left_drive_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/gamepad/left_drive", 10, std::bind(&Gamepad_to_Motor::left_joystick_callback, this, _1)
    );
    this->right_drive_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/gamepad/right_drive", 10, std::bind(&Gamepad_to_Motor::right_joystick_callback, this, _1)
    );
    // this->button_a_sub = this->create_subscription<std_msgs::msg::Bool>(
    //   "/gamepad/exc_intrnl_mtr", 10, std::bind(&Gamepad_to_Motor::_callback, this, _1)
    // );
    this->button_b_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/gamepad/exc_thrd_rod_actr", 10, std::bind(&Gamepad_to_Motor::hard_stop_callback, this, _1)
    );
    this->button_lb_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/gamepad/left_back_cltn_mtr", 10, std::bind(&Gamepad_to_Motor::point_turn_left_callback, this, _1)
    );
    this->button_rb_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/gamepad/right_back_cltn_mtr", 10, std::bind(&Gamepad_to_Motor::point_turn_right_callback, this, _1)
    );
  
  } catch (const std::exception &e) {
    std::cerr << "Error w/ Initializing joy_to_tank_control Subscribers: " << e.what() << std::endl;
    throw std::runtime_error("Error w/ Initializing joy_to_tank_control Subscribers");
  }

  try {
      // Enable and run motor
      this->tank_controller.setMotors(0.1, 0.1, 5);
      this->tank_controller.turn(90, 0.1);
      std::cout << "Startup sequence/test complete" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(3));
  } catch (const std::exception &e) {
      std::cerr << "Error w/ Initializing and/or Starting up TankController: " << e.what() << std::endl;
      throw std::runtime_error("Error w/ Initializing and/or Starting up TankController");
  }

  // Initialize Caches
  this->last_command_left_joy = 0;
  this->last_command_right_joy = 0;
  this->killed = false;
}

void Gamepad_to_Motor::left_joystick_callback(const std_msgs::msg::Float32& msg) {
  if (!this->killed) {

    float left_prefilter = msg.data;
    this->last_command_left_joy = left_prefilter;
    float right_prefilter = this->last_command_right_joy;

    float left =  (left_prefilter < DEADZONE || left_prefilter > (1 - DEADZONE)) ? 
                  (left_prefilter < DEADZONE ? 0 : 1) : left_prefilter;

    float right = (right_prefilter < DEADZONE || right_prefilter > (1 - DEADZONE)) ? 
                  (right_prefilter < DEADZONE ? 0 : 1) : right_prefilter;

    double throttle = this->joystick_to_throttle(left, right);
    double turn = this->joystick_to_turn(left, right);

    this->tank_controller.drive(throttle, turn, 0.1);

  }
  return;
}

void Gamepad_to_Motor::right_joystick_callback(const std_msgs::msg::Float32& msg) {
  if (!this->killed) {

    float right_prefilter = msg.data;
    this->last_command_right_joy = right_prefilter;
    float left_prefilter = this->last_command_left_joy;

    float left =  (left_prefilter < DEADZONE || left_prefilter > (1 - DEADZONE)) ? 
                  (left_prefilter < DEADZONE ? 0 : 1) : left_prefilter;

    float right = (right_prefilter < DEADZONE || right_prefilter > (1 - DEADZONE)) ? 
                  (right_prefilter < DEADZONE ? 0 : 1) : right_prefilter;

    double throttle = this->joystick_to_throttle(left, right);
    double turn = this->joystick_to_turn(left, right);

    this->tank_controller.drive(throttle, turn, 0.1);

  }
  return;
}

void Gamepad_to_Motor::point_turn_left_callback(const std_msgs::msg::Bool& msg) {
  if (!this->killed && msg.data) { this->tank_controller.turn(-90, 0.5); }
  return;
}

void Gamepad_to_Motor::point_turn_right_callback(const std_msgs::msg::Bool& msg) {
  if (!this->killed && msg.data) { this->tank_controller.turn(90, 0.5); }
  return;
}

void Gamepad_to_Motor::hard_stop_callback(const std_msgs::msg::Bool& msg) {
  /* TODO: Implement Opposing Current Velocity Vector */

  if (!this->killed && msg.data) { this->tank_controller.stop(1); }
  return;
}

double Gamepad_to_Motor::joystick_to_throttle(float left, float right) {
  return static_cast<double>(0.5 * (left + right));
}
    
double Gamepad_to_Motor::joystick_to_turn(float left, float right) {
  return static_cast<double>(left - right);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gamepad_to_Motor>());
  rclcpp::shutdown();
  return 0;
}
