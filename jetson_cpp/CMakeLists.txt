cmake_minimum_required(VERSION 3.5)
project(jetson_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/sparkcan sparkcan)

add_executable(jetson_sparkmax "src/jetson_sparkmax.cpp")
target_include_directories(jetson_sparkmax PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/sparkcan/include)
ament_target_dependencies(jetson_sparkmax rclcpp std_msgs)
target_link_libraries(jetson_sparkmax sparkcan)

install(TARGETS jetson_sparkmax DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
