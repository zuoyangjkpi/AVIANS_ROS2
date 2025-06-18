cmake_minimum_required(VERSION 3.8)
project(ros2_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(neural_network_msgs REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories (CRITICAL: This makes headers available)
include_directories(include)

# Create src directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add executable for timestamp monitor
add_executable(timestamp_monitor_node
  src/timestamp_monitor.cpp
)

# Set include directories for the target
target_include_directories(timestamp_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(timestamp_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  neural_network_msgs
  uav_msgs
  nav_msgs
)

# Install the executable
install(TARGETS
  timestamp_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (existing)
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Add to the install section
install(PROGRAMS
  scripts/analyze_timing.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/
  DESTINATION share/${PROJECT_NAME}/scripts/
  USE_SOURCE_PERMISSIONS
)

# Create data directory during install
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/data/timing_logs
)

# Export dependencies for other packages (existing)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()