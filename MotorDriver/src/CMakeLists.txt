cmake_minimum_required(VERSION 3.10)
project(tank_control VERSION 1.5 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the necessary packages
find_package(
    sparkcan REQUIRED
)

# Define the executables and link with the sparkcan library
add_executable(example_control src/control.cpp)
add_executable(example_status src/status.cpp)
add_executable(tank_control src/tank_control.cpp)

target_link_libraries(example_control PRIVATE ${sparkcan_LIBRARIES})
target_link_libraries(example_status PRIVATE ${sparkcan_LIBRARIES})
target_link_libraries(tank_control PRIVATE ${sparkcan_LIBRARIES})
