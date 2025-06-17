# ROS2 Utilities Package

This package provides shared utilities for ROS2 nodes in simulation environments.

## Clock Synchronization

The `ClockSynchronizer` class ensures proper time synchronization in simulation:

```cpp
#include <ros2_utils/clock_sync.hpp>

// In your node's main function:
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    
    // Wait for simulation clock
    WAIT_FOR_CLOCK_DELAYED(node);
    
    rclcpp::spin(node);
    return 0;
}
```

## Requirements

- Set `use_sim_time: true` in launch files for simulation
- Ensure Gazebo or simulator publishes `/clock` topic
- Call clock sync after node construction but before spinning

# =============================================================================
# 6. UPDATE EXISTING PACKAGES TO USE ros2_utils
# =============================================================================

# Add dependency to each package's package.xml:
<depend>ros2_utils</depend>

# Add dependency to each package's CMakeLists.txt:
find_package(ros2_utils REQUIRED)

ament_target_dependencies(your_executable
  rclcpp
  ros2_utils
  # ... other dependencies
)

# Include in your source files:
#include <ros2_utils/clock_sync.hpp>

# Use in your main functions:
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNodeClass>();
    
    WAIT_FOR_CLOCK_DELAYED(node);
    
    rclcpp::spin(node);
    return 0;
}