/**
 * @file joy_to_tank_control.cpp
 * @author Rushil Shah
 * 
 * @brief Integration b/w Xbox Controller & Motors via Tank Control Class
 */

#include "joy_to_tank_control.hpp"

Gamepad_to_Motor::Gamepad_to_Motor() : Node("gamepad_to_motor") {
  left_drive_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/gamepad/left_drive", 10, std::bind(&Gamepad_to_Motor::topic_callback, this, _1));
  right_drive_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/gamepad/right_drive", 10, std::bind(&Gamepad_to_Motor::topic_callback, this, _1));

  std::cout << "Yo, what up" << std::endl;
}

void Gamepad_to_Motor::topic_callback(const std_msgs::msg::Float32& msg) const {
  float_t float32 = msg.data;
  std::cout << float32 << std::endl;
  return;
}
    

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gamepad_to_Motor>());
  rclcpp::shutdown();
  return 0;
}
