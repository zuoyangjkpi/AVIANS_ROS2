cmake_minimum_required(VERSION 3.8)
project(drone_state_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_utils REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(drone_state_publisher_node
  src/drone_state_publisher.cpp
)

# Link dependencies
ament_target_dependencies(drone_state_publisher_node
  rclcpp
  nav_msgs
  geometry_msgs
  uav_msgs
  tf2
  tf2_geometry_msgs
  ros2_utils
)

# Install targets
install(TARGETS
  drone_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install config files if any
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()