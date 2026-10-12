cmake_minimum_required(VERSION 3.8)
project(driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/../dependencies/sparkcan ${CMAKE_BINARY_DIR}/sparkcan_build)
 include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../dependencies/sparkcan/include
)

add_executable(driver src/driver_node.cpp)
ament_target_dependencies(driver PUBLIC rclcpp std_msgs)
target_link_libraries(driver PUBLIC sparkcan)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
