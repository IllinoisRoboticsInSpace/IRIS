cmake_minimum_required(VERSION 3.10)
project(sparkcan VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define library targets as shared
add_library(sparkcan SHARED
    src/PIDController.cpp
    src/SparkBase.cpp
    src/SparkMax.cpp 
    src/SparkFlex.cpp
)

# Set include directories
target_include_directories(sparkcan PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Define the executables
add_executable(example_control examples/control.cpp)
add_executable(example_pid examples/pid.cpp)
add_executable(example_status examples/status.cpp)

# Link the executables with the sparkmax-can library
target_link_libraries(example_control PRIVATE sparkcan)
target_link_libraries(example_pid PRIVATE sparkcan)
target_link_libraries(example_status PRIVATE sparkcan)

# Install the library
install(TARGETS sparkcan
    EXPORT sparkcanTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the header files
install(DIRECTORY include/ DESTINATION include)

# Export the targets
install(EXPORT sparkcanTargets
    FILE sparkcanTargets.cmake
    NAMESPACE sparkcan::
    DESTINATION lib/cmake/sparkcan
)

# Create and install the package configuration file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sparkcanConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/sparkcanConfig.cmake"
    INSTALL_DESTINATION lib/cmake/sparkcan
)

# Generate the version file for the package
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/sparkcanConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install the config and version files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/sparkcanConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/sparkcanConfigVersion.cmake"
    DESTINATION lib/cmake/sparkcan
)
