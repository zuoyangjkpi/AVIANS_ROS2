#ifndef PERSON_TRACKER_PROJECTION_NODE_HPP
#define PERSON_TRACKER_PROJECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/opencv.hpp>

#include "neural_network_detector/msg/neural_network_detection_array.hpp"
#include "neural_network_detector/msg/neural_network_feedback.hpp"

namespace person_tracker_projection
{

struct CameraModel
{
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    cv::Size image_size;
    bool initialized = true;
};

class PersonTrackerProjectionNode : public rclcpp::Node
{
public:
    explicit PersonTrackerProjectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Callback functions
    void detectionsCallback(const neural_network_detector::msg::NeuralNetworkDetectionArray::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Core functionality
    geometry_msgs::msg::PoseStamped projectDetectionToWorld(
        const neural_network_detector::msg::NeuralNetworkDetection& detection,
        const std_msgs::msg::Header& header);
    
    neural_network_detector::msg::NeuralNetworkFeedback generateFeedback(
        const neural_network_detector::msg::NeuralNetworkDetection& detection);
    
    cv::Point2f pixelToNormalizedImageCoordinates(const cv::Point2f& pixel);
    geometry_msgs::msg::Point projectToGround(const cv::Point2f& normalized_point, 
                                             double camera_height);
    
    bool transformPoseToTargetFrame(geometry_msgs::msg::PoseStamped& pose);
    
    // Parameter initialization
    void initializeParameters();
    
    // Subscribers
    rclcpp::Subscription<neural_network_detector::msg::NeuralNetworkDetectionArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<neural_network_detector::msg::NeuralNetworkFeedback>::SharedPtr feedback_pub_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Camera model
    CameraModel camera_model_;
    
    // Current drone state
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    
    // Parameters
    std::string camera_frame_;
    std::string target_frame_;
    std::string drone_frame_;
    double assumed_person_height_;
    double min_detection_confidence_;
    int target_person_class_;
    double feedback_margin_ratio_;
    double tracking_offset_distance_;
    double tracking_offset_height_;
    bool publish_feedback_;
    
    // State tracking
    rclcpp::Time last_detection_time_;
    geometry_msgs::msg::PoseStamped last_valid_waypoint_;
    bool has_valid_waypoint_;
};

} // namespace person_tracker_projection

#endif // PERSON_TRACKER_PROJECTION_NODE_HPP