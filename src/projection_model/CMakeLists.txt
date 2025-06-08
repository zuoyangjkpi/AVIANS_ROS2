cmake_minimum_required(VERSION 3.8)
project(projection_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(neural_network_detector REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)

# Create person tracker projection library
add_library(person_tracker_projection_component SHARED
  src/person_tracker_projection_node.cpp
)

target_compile_features(person_tracker_projection_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(person_tracker_projection_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(person_tracker_projection_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  neural_network_detector
  cv_bridge
  OpenCV
  nav_msgs
)

# Register the component
rclcpp_components_register_nodes(person_tracker_projection_component 
  "person_tracker_projection::PersonTrackerProjectionNode")

# Create executable
add_executable(person_tracker_projection_node
  src/main.cpp
)

target_link_libraries(person_tracker_projection_node 
  person_tracker_projection_component
)

ament_target_dependencies(person_tracker_projection_node
  rclcpp
)

# Install targets
install(TARGETS
  person_tracker_projection_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  person_tracker_projection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(person_tracker_projection_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  neural_network_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  cv_bridge
  nav_msgs
  OpenCV
)

ament_package()