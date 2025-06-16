//
// ROS2 Main executable for DistributedKF3D
//

#include <rclcpp/rclcpp.hpp>
#include <target_tracker_distributed_kf/DistributedKF3D.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<target_tracker_distributed_kf::DistributedKF3D>();
    
    RCLCPP_INFO(node->get_logger(), "DistributedKF3D node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}