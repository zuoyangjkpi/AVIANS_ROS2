cmake_minimum_required(VERSION 3.8)
project(neural_network_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(include)
include_directories(third_party/YOLOs-CPP/include)  # Add YOLO include path
include_directories(third_party/YOLOs-CPP/onnxruntime-linux-x64-1.20.1/include)  # Add ONNX include path

set(msg_files
  "msg/NeuralNetworkDetection.msg"  
  "msg/NeuralNetworkDetectionArray.msg"
  "msg/NeuralNetworkFeedback.msg"
  "msg/NeuralNetworkNumberOfDetections.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES  std_msgs
)

add_executable(yolo12_detector_node 
src/yolo12_detector_node.cpp
)
ament_target_dependencies( yolo12_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
  rclcpp_components
)

# set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/YOLOs-CPP/onnxruntime-linux-x64-1.20.1")

# # Verify the library exists
# if(NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
#     message(FATAL_ERROR "ONNX Runtime library not found at: ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
# endif()


# Add the ONNX Runtime lib directory to linker search path
# link_directories(${ONNXRUNTIME_ROOT}/lib)

# Link ONNX Runtime libraries to your executable
target_link_libraries(yolo12_detector_node 
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/YOLOs-CPP/onnxruntime-linux-x64-1.20.1/lib/libonnxruntime.so.1.20.1"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(yolo12_detector_node "${cpp_typesupport_target}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the executable
install(TARGETS
  yolo12_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
