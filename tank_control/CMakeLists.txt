cmake_minimum_required(VERSION 3.10)
project(tank_control VERSION 1.5 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the necessary packages
find_package(sparkcan REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(joy_to_tank_control src/joy_to_tank_control.cpp)
add_executable(tank_control src/tank_control.cpp)

# Link executables against libraries
target_include_directories(joy_to_tank_control PUBLIC include)
target_link_libraries(tank_control PRIVATE ${sparkcan_LIBRARIES})
target_include_directories(tank_control PUBLIC include)

ament_target_dependencies(joy_to_tank_control rclcpp std_msgs)

install(TARGETS
  joy_to_tank_control
  tank_control
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
