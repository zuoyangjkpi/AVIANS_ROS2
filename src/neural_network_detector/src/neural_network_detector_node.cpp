#include <rclcpp/rclcpp.hpp>
#include "neural_network_detector/yolo12_detector_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        // Pass NodeOptions explicitly
        auto node = std::make_shared<yolo12_detector_node::YOLO12DetectorNode>(
            rclcpp::NodeOptions()
        );
        
        RCLCPP_INFO(node->get_logger(), "Starting YOLO12 Detector Node...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo12_detector_node"), 
                     "Failed to start node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}