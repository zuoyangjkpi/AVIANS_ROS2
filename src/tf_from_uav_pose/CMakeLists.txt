cmake_minimum_required(VERSION 3.8)
project(tf_from_uav_pose_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(uav_msgs REQUIRED)

# Note: MRPT dependencies removed to avoid compatibility issues

# Include directories
include_directories(include)

# Create executable
add_executable(tf_from_uav_pose_node
  src/tf_from_uav_pose.cpp
)

# Link dependencies
ament_target_dependencies(tf_from_uav_pose_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  uav_msgs
)

# Install targets
install(TARGETS
  tf_from_uav_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()