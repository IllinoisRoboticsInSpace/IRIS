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

add_subdirectory(${CMAKE_SOURCE_DIR}/../dependencies/sparkcan ${CMAKE_BINARY_DIR}/sparkcan_build)
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../dependencies/sparkcan/include
)

# Add tank control library
add_library(tank_control_lib src/tank_control.cpp)

# Add executables
add_executable(joy_to_tank_control src/joy_to_tank_control.cpp)
add_executable(cytron_linear_actuator src/cytron_linear_actuator.cpp)

# Link executables against libraries
target_link_libraries(sparkcan)
target_link_libraries(tank_control_lib PRIVATE ${sparkcan_LIBRARIES})
target_include_directories(tank_control_lib PUBLIC include)
target_link_libraries(joy_to_tank_control PRIVATE ${sparkcan_LIBRARIES})
target_link_libraries(joy_to_tank_control PRIVATE tank_control_lib)
target_include_directories(joy_to_tank_control PUBLIC include)
target_link_libraries(cytron_linear_actuator PUBLIC CytronMotorDriver)

ament_target_dependencies(joy_to_tank_control PUBLIC rclcpp std_msgs)
ament_target_dependencies(cytron_linear_actuator PUBLIC rclcpp std_msgs)

install(TARGETS
  joy_to_tank_control
  tank_control_lib
  cytron_linear_actuator
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS resources/canable_start.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
