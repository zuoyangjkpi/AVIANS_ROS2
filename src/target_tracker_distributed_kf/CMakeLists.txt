cmake_minimum_required(VERSION 3.8)
project(target_tracker_distributed_kf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(pose_cov_ops REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros2_utils REQUIRED)

# Find MRPT
# find_package(mrpt-math REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/DistributedKF3D.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  uav_msgs
  pose_cov_ops
  tf2
  tf2_geometry_msgs
  rcl_interfaces
  ros2_utils
)

target_link_libraries(${PROJECT_NAME}
  # mrpt::math
  Eigen3::Eigen
)

add_executable(distributed_kf_node src/main.cpp)

target_link_libraries(distributed_kf_node
  ${PROJECT_NAME}
)

ament_target_dependencies(distributed_kf_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  distributed_kf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  uav_msgs
  pose_cov_ops
  tf2
  tf2_geometry_msgs
  rcl_interfaces
  ros2_utils
)

ament_export_include_directories(include)

ament_package()