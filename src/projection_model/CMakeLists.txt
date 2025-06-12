cmake_minimum_required(VERSION 3.8)
project(projection_model)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(neural_network_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pose_cov_ops_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(${PROJECT_NAME}_node
  src/Projector.cpp
  src/models/Model3D.cpp
)

# Add library for the projector class (optional, for reusability)
add_library(${PROJECT_NAME}
  src/Projector.cpp
)

# Dependencies for the executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  sensor_msgs
  neural_network_msgs
  image_geometry
  cv_bridge
  OpenCV
  tf2_geometry_msgs
  pose_cov_ops_interface
  rcl_interfaces
)

# Dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  neural_network_msgs
  image_geometry
  cv_bridge
  OpenCV
  tf2_geometry_msgs
  pose_cov_ops_interface
  rcl_interfaces
)

# Link OpenCV libraries
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  FILES_MATCHING PATTERN "*.yaml"
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  neural_network_msgs
  image_geometry
  cv_bridge
  OpenCV
  tf2_geometry_msgs
  pose_cov_ops
  rcl_interfaces
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()