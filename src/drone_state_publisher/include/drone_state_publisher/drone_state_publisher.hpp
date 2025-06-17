#ifndef DRONE_STATE_PUBLISHER_HPP
#define DRONE_STATE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <uav_msgs/msg/uav_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace drone_state_publisher {

class DroneStatePublisher : public rclcpp::Node {
public:
    DroneStatePublisher();

private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr target_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr target_offset_sub_;
    
    // Publishers
    rclcpp::Publisher<uav_msgs::msg::UAVPose>::SharedPtr uav_pose_pub_;          // /machine_1/pose (with offset)
    rclcpp::Publisher<uav_msgs::msg::UAVPose>::SharedPtr uav_raw_pose_pub_;      // /machine_1/pose/raw (without offset)
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    
    // Stored states
    nav_msgs::msg::Odometry::SharedPtr current_drone_odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_target_pose_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr current_target_twist_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_target_offset_;
    
    // Parameters for optimal viewing
    double optimal_distance_;      // Distance from target
    double optimal_height_offset_; // Height above target
    double optimal_angle_offset_;  // Angle offset for side viewing
    
    // Constant covariance values
    std::array<double, 100> constant_covariance_;
    
    // Callbacks
    void droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void targetPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void targetTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void targetOffsetCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    // Helper functions
    void publishUAVPose();
    void publishRawUAVPose();
    void publishTargetWaypoint();
    geometry_msgs::msg::PoseStamped calculateOptimalViewingPose();
    void initializeConstantCovariance();
};

} // namespace drone_state_publisher

#endif // DRONE_STATE_PUBLISHER_HPP