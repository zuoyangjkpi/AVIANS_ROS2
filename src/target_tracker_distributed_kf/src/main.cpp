//
// ROS2 Main executable for DistributedKF3D
// FIXED: Safe initialization pattern to avoid shared_from_this in constructor
//

#include <rclcpp/rclcpp.hpp>
#include <target_tracker_distributed_kf/DistributedKF3D.h>
#include <ros2_utils/clock_sync.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create the node instance
    auto node = std::make_shared<target_tracker_distributed_kf::DistributedKF3D>();
    
    RCLCPP_INFO(node->get_logger(), "DistributedKF3D node created");

    // CRITICAL: Wait for clock synchronization BEFORE calling initialize()
    WAIT_FOR_CLOCK_DELAYED(node);

    // NOW it's safe to call initialize() which may use shared_from_this
    node->initialize();

    RCLCPP_INFO(node->get_logger(), "DistributedKF3D node started with synchronized clock");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}