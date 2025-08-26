cmake_minimum_required(VERSION 3.22)
project(neural_network_detector)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(neural_network_msgs REQUIRED)

# ONNX Runtime
set(ONNXRUNTIME_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YOLOs-CPP/onnxruntime-linux-x64-1.20.1)
set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_ROOT_PATH}/include)
set(ONNXRUNTIME_LIB ${ONNXRUNTIME_ROOT_PATH}/lib/libonnxruntime.so.1.20.1)

# Check if ONNX Runtime exists
if(NOT EXISTS ${ONNXRUNTIME_LIB})
 message(FATAL_ERROR "ONNX Runtime library not found at: 
${ONNXRUNTIME_LIB}")
endif()

# Include directories
include_directories(
 include
 ${ONNXRUNTIME_INCLUDE_DIRS}
 third_party/YOLOs-CPP/include
)

# YOLO12 detector node
add_executable(yolo12_detector_node 
 src/yolo12_detector_node.cpp
 third_party/YOLOs-CPP/src/YOLO12.cpp
)
ament_target_dependencies(yolo12_detector_node
 rclcpp
 sensor_msgs
 geometry_msgs
 cv_bridge
 OpenCV
 neural_network_msgs
)
target_link_libraries(yolo12_detector_node
 ${ONNXRUNTIME_LIB}
)
# Neural network detector node (original)
add_executable(neural_network_detector_node 
src/neural_network_detector_node.cpp)
ament_target_dependencies(neural_network_detector_node
 rclcpp
 sensor_msgs
 geometry_msgs
 cv_bridge
 OpenCV
 neural_network_msgs
)
# Install targets
install(TARGETS
 yolo12_detector_node
 neural_network_detector_node
 DESTINATION lib/${PROJECT_NAME}
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