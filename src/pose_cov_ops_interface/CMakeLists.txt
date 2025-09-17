cmake_minimum_required(VERSION 3.8)
project(pose_cov_ops_interface)

set(CMAKE_CXX_STANDARD 17)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pose_cov_ops REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Library
add_library(${PROJECT_NAME}
  src/pose_cov_ops_interface.cpp
  include/pose_cov_ops_interface/pose_cov_ops_interface.h
)

include_directories(include)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pose_cov_ops
  message_filters
  geometry_msgs
)

# Executable
add_executable(pose_cov_ops_interface_node src/pose_cov_ops_interface_node.cpp)

ament_target_dependencies(pose_cov_ops_interface_node
  rclcpp
  geometry_msgs
  nav_msgs
)


# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pose_cov_ops_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  message_filters
  pose_cov_ops
)


# Add test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_cov_ops
    test/test_pose_cov_ops_interface.cpp
  )

  # target_link_libraries(test_pose_cov_ops
  # ${PROJECT_NAME}
  # rclcpp::rclcpp
  # )


  target_link_libraries(test_pose_cov_ops
  ${PROJECT_NAME}
  rclcpp::rclcpp
  curl
)

install(TARGETS test_pose_cov_ops
  DESTINATION lib/${PROJECT_NAME}
)
  
endif()

ament_package()
