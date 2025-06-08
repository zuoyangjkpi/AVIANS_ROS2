#include <rclcpp/rclcpp.hpp>
#include "projection_model/person_tracker_projection_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<person_tracker_projection::PersonTrackerProjectionNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}