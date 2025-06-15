cmake_minimum_required(VERSION 3.8)
project(tf_from_uav_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)



# Create library
add_library(${PROJECT_NAME} 
  src/tf_from_uav_pose.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  uav_msgs
  rcl_interfaces
  Eigen3
)

# Create executable
add_executable(${PROJECT_NAME}_node 
  src/tf_from_uav_pose.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  uav_msgs
  rcl_interfaces
  Eigen3
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${Eigen3_INCLUDE_DIRS}
)


# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
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