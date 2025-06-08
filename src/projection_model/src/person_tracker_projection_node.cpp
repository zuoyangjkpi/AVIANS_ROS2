#include "projection_model/person_tracker_projection_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

namespace person_tracker_projection
{

PersonTrackerProjectionNode::PersonTrackerProjectionNode(const rclcpp::NodeOptions & options)
: Node("person_tracker_projection_node", options),
  has_valid_waypoint_(false)
{
    // Initialize parameters
    initializeParameters();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create subscribers
    detections_sub_ = this->create_subscription<neural_network_detector::msg::NeuralNetworkDetectionArray>(
        "detections", 
        rclcpp::QoS(10),
        std::bind(&PersonTrackerProjectionNode::detectionsCallback, this, std::placeholders::_1));
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info",
        rclcpp::QoS(1).transient_local(),
        std::bind(&PersonTrackerProjectionNode::cameraInfoCallback, this, std::placeholders::_1));
    
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/X3/odom",
        rclcpp::QoS(10),
        std::bind(&PersonTrackerProjectionNode::droneOdomCallback, this, std::placeholders::_1));
    
    // Create publishers
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target_waypoint", rclcpp::QoS(10));
    
    if (publish_feedback_) {
        feedback_pub_ = this->create_publisher<neural_network_detector::msg::NeuralNetworkFeedback>(
            "feedback", rclcpp::QoS(10));
    }
    
    RCLCPP_INFO(this->get_logger(), "Person Tracking Projection Node initialized successfully");
}

void PersonTrackerProjectionNode::initializeParameters()
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    this->declare_parameter<std::string>("target_frame", "map");
    this->declare_parameter<std::string>("drone_frame", "base_link");
    this->declare_parameter<double>("assumed_person_height", 1.7);
    this->declare_parameter<double>("min_detection_confidence", 0.5);
    this->declare_parameter<int>("target_person_class", 0);
    this->declare_parameter<double>("feedback_margin_ratio", 0.1);
    this->declare_parameter<double>("tracking_offset_distance", 3.0);
    this->declare_parameter<double>("tracking_offset_height", 1.0);
    this->declare_parameter<bool>("publish_feedback", true);
    
    // Get parameter values
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    drone_frame_ = this->get_parameter("drone_frame").as_string();
    assumed_person_height_ = this->get_parameter("assumed_person_height").as_double();
    min_detection_confidence_ = this->get_parameter("min_detection_confidence").as_double();
    target_person_class_ = this->get_parameter("target_person_class").as_int();
    feedback_margin_ratio_ = this->get_parameter("feedback_margin_ratio").as_double();
    tracking_offset_distance_ = this->get_parameter("tracking_offset_distance").as_double();
    tracking_offset_height_ = this->get_parameter("tracking_offset_height").as_double();
    publish_feedback_ = this->get_parameter("publish_feedback").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Camera frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Target frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Assumed person height: %.2f m", assumed_person_height_);
    RCLCPP_INFO(this->get_logger(), "  Tracking offset: %.2f m distance, %.2f m height", 
                tracking_offset_distance_, tracking_offset_height_);
}

void PersonTrackerProjectionNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null CameraInfo message");
        return;
    }
    
    if (camera_model_.initialized) {
        return; // Already initialized
    }
    
    // Check if K array has expected size
    if (msg->k.size() < 9) {
        RCLCPP_ERROR(this->get_logger(), "CameraInfo K matrix has insufficient elements: %zu", msg->k.size());
        return;
    }
    
    // Initialize camera model from CameraInfo message
    camera_model_.camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
    camera_model_.camera_matrix.at<double>(0, 0) = msg->k[0]; // fx
    camera_model_.camera_matrix.at<double>(0, 2) = msg->k[2]; // cx
    camera_model_.camera_matrix.at<double>(1, 1) = msg->k[4]; // fy
    camera_model_.camera_matrix.at<double>(1, 2) = msg->k[5]; // cy
    camera_model_.camera_matrix.at<double>(2, 2) = 1.0;
    
    camera_model_.distortion_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i = 0; i < 5 && i < msg->d.size(); ++i) {
        camera_model_.distortion_coeffs.at<double>(0, i) = msg->d[i];
    }
    
    camera_model_.image_size = cv::Size(msg->width, msg->height);
    camera_model_.initialized = true;
    
    RCLCPP_INFO(this->get_logger(), "Camera model initialized: %dx%d, fx=%.1f, fy=%.1f", 
                msg->width, msg->height, msg->k[0], msg->k[4]);
}

void PersonTrackerProjectionNode::droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null Odometry message");
        return;
    }
    current_odom_ = msg;
}

void PersonTrackerProjectionNode::detectionsCallback(
    const neural_network_detector::msg::NeuralNetworkDetectionArray::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null DetectionArray message");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received %zu detections", msg->detections.size());
    
    if (!camera_model_.initialized) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Camera model not initialized yet");
        return;
    }
    
    if (!current_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "No drone odometry available yet");
        return;
    }
    
    // Find the best detection (highest confidence person)
    neural_network_detector::msg::NeuralNetworkDetection best_detection;
    bool found_person = false;
    double highest_confidence = 0.0;
    
    for (const auto& detection : msg->detections) {
        RCLCPP_INFO(this->get_logger(), "Detection: class=%d, confidence=%.3f, bounds=[%d,%d,%d,%d]", 
                   detection.object_class, detection.detection_score,
                   detection.xmin, detection.ymin, detection.xmax, detection.ymax);
        
        // Validate detection bounds
        if (detection.xmin < 0 || detection.xmax < 0 || 
            detection.ymin < 0 || detection.ymax < 0 ||
            detection.xmax <= detection.xmin || detection.ymax <= detection.ymin) {
            RCLCPP_WARN(this->get_logger(), "Invalid detection bounds: [%d,%d,%d,%d]", 
                       detection.xmin, detection.ymin, detection.xmax, detection.ymax);
            continue;
        }
        
        // Debug filtering criteria
        RCLCPP_INFO(this->get_logger(), "Checking filters: class=%d (target=%d), confidence=%.3f (min=%.3f)", 
                   detection.object_class, target_person_class_,
                   detection.detection_score, min_detection_confidence_);
        
        // Filter by class and confidence
        if (detection.object_class == target_person_class_ && 
            detection.detection_score >= min_detection_confidence_ &&
            detection.detection_score > highest_confidence) {
            
            RCLCPP_INFO(this->get_logger(), "Found valid person detection with confidence %.3f", 
                       detection.detection_score);
            best_detection = detection;
            highest_confidence = detection.detection_score;
            found_person = true;
        }
    }
    
    if (found_person) {
        RCLCPP_INFO(this->get_logger(), "Processing best detection with confidence %.3f", highest_confidence);
        
        // Project detection to world coordinates
        auto waypoint = projectDetectionToWorld(best_detection, msg->header);
        
        if (waypoint.header.frame_id != "") {
            // Publish waypoint
            if (waypoint_pub_) {
                waypoint_pub_->publish(waypoint);
                RCLCPP_INFO(this->get_logger(), 
                           "Published waypoint: [%.2f, %.2f, %.2f] with confidence %.2f",
                           waypoint.pose.position.x, waypoint.pose.position.y, 
                           waypoint.pose.position.z, highest_confidence);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Waypoint publisher is null!");
            }
            
            last_valid_waypoint_ = waypoint;
            has_valid_waypoint_ = true;
            last_detection_time_ = this->get_clock()->now();
            
            // Generate and publish feedback
            if (publish_feedback_ && feedback_pub_) {
                auto feedback = generateFeedback(best_detection);
                feedback.header = msg->header;
                feedback_pub_->publish(feedback);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to project detection to world coordinates");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "No valid person detections found (checked %zu detections)", 
                   msg->detections.size());
    }
}

geometry_msgs::msg::PoseStamped PersonTrackerProjectionNode::projectDetectionToWorld(
    const neural_network_detector::msg::NeuralNetworkDetection& detection,
    const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = header;
    
    try {
        RCLCPP_INFO(this->get_logger(), "Starting projection for detection [%d,%d,%d,%d]",
                   detection.xmin, detection.ymin, detection.xmax, detection.ymax);
        
        // Validate detection bounds against image size
        if (detection.xmax > camera_model_.image_size.width || 
            detection.ymax > camera_model_.image_size.height) {
            RCLCPP_WARN(this->get_logger(), "Detection bounds [%d,%d] exceed image size [%d,%d]",
                       detection.xmax, detection.ymax, 
                       camera_model_.image_size.width, camera_model_.image_size.height);
            waypoint.header.frame_id = "";
            return waypoint;
        }
        
        // Calculate person's foot position (bottom center of bounding box)
        cv::Point2f foot_pixel(
            (detection.xmin + detection.xmax) / 2.0f,
            detection.ymax
        );
        
        RCLCPP_INFO(this->get_logger(), "Foot pixel: [%.1f, %.1f]", foot_pixel.x, foot_pixel.y);
        
        // Convert to normalized image coordinates
        cv::Point2f normalized_point = pixelToNormalizedImageCoordinates(foot_pixel);
        
        RCLCPP_INFO(this->get_logger(), "Normalized point: [%.3f, %.3f]", 
                   normalized_point.x, normalized_point.y);
        
        // Get camera height from current drone position
        double camera_height = current_odom_->pose.pose.position.z;
        
        RCLCPP_INFO(this->get_logger(), "Camera height: %.2f", camera_height);
        
        // Validate camera height
        if (camera_height <= 0.1) {
            RCLCPP_WARN(this->get_logger(), "Invalid camera height: %.2f", camera_height);
            waypoint.header.frame_id = "";
            return waypoint;
        }
        
        // Project to ground plane
        geometry_msgs::msg::Point ground_point = projectToGround(normalized_point, camera_height);
        
        RCLCPP_INFO(this->get_logger(), "Ground point in %s: [%.2f, %.2f, %.2f]", 
                   camera_frame_.c_str(), ground_point.x, ground_point.y, ground_point.z);
        
        // Create pose in camera frame
        waypoint.pose.position = ground_point;
        waypoint.pose.orientation.w = 1.0; // No rotation
        waypoint.header.frame_id = camera_frame_;
        
        // Transform to target frame if needed
        if (transformPoseToTargetFrame(waypoint)) {
            RCLCPP_INFO(this->get_logger(), "Transformed to %s: [%.2f, %.2f, %.2f]", 
                       target_frame_.c_str(), 
                       waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
            
            // Add tracking offset (stay behind and above the person)
            waypoint.pose.position.z = std::max(0.0, waypoint.pose.position.z + tracking_offset_height_);
            
            // Calculate offset direction (opposite to person's position relative to drone)
            if (current_odom_) {
                double dx = waypoint.pose.position.x - current_odom_->pose.pose.position.x;
                double dy = waypoint.pose.position.y - current_odom_->pose.pose.position.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                RCLCPP_INFO(this->get_logger(), "Drone at [%.2f, %.2f], person at [%.2f, %.2f], distance: %.2f",
                           current_odom_->pose.pose.position.x, current_odom_->pose.pose.position.y,
                           waypoint.pose.position.x, waypoint.pose.position.y, distance);
                
                if (distance > 0.1) { // Avoid division by zero
                    double offset_x = -(dx / distance) * tracking_offset_distance_;
                    double offset_y = -(dy / distance) * tracking_offset_distance_;
                    waypoint.pose.position.x += offset_x;
                    waypoint.pose.position.y += offset_y;
                    
                    RCLCPP_INFO(this->get_logger(), "Applied offset [%.2f, %.2f], final waypoint: [%.2f, %.2f, %.2f]",
                               offset_x, offset_y,
                               waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform pose to target frame");
            waypoint.header.frame_id = "";
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error projecting detection to world: %s", e.what());
        waypoint.header.frame_id = "";
    }
    
    return waypoint;
}

cv::Point2f PersonTrackerProjectionNode::pixelToNormalizedImageCoordinates(const cv::Point2f& pixel)
{
    // Convert pixel coordinates to normalized image coordinates
    cv::Point2f normalized;
    
    // Validate camera matrix is properly initialized
    if (camera_model_.camera_matrix.empty() || camera_model_.camera_matrix.rows != 3 || 
        camera_model_.camera_matrix.cols != 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera matrix dimensions");
        return normalized; // Return (0,0) 
    }
    
    double fx = camera_model_.camera_matrix.at<double>(0, 0);
    double fy = camera_model_.camera_matrix.at<double>(1, 1);
    double cx = camera_model_.camera_matrix.at<double>(0, 2);
    double cy = camera_model_.camera_matrix.at<double>(1, 2);
    
    // Check for invalid focal lengths
    if (std::abs(fx) < 1e-6 || std::abs(fy) < 1e-6) {
        RCLCPP_ERROR(this->get_logger(), "Invalid focal lengths: fx=%.6f, fy=%.6f", fx, fy);
        return normalized; // Return (0,0)
    }
    
    normalized.x = (pixel.x - cx) / fx;
    normalized.y = (pixel.y - cy) / fy;
    
    return normalized;
}

geometry_msgs::msg::Point PersonTrackerProjectionNode::projectToGround(
    const cv::Point2f& normalized_point, double camera_height)
{
    geometry_msgs::msg::Point ground_point;
    
    // Simple pinhole projection to ground plane
    // Assumes camera is looking straight down (or nearly so)
    ground_point.x = normalized_point.x * camera_height;
    ground_point.y = normalized_point.y * camera_height;
    ground_point.z = 0.0; // Ground level
    
    return ground_point;
}

bool PersonTrackerProjectionNode::transformPoseToTargetFrame(geometry_msgs::msg::PoseStamped& pose)
{
    if (pose.header.frame_id == target_frame_) {
        return true;
    }
    
    if (!tf_buffer_) {
        RCLCPP_ERROR(this->get_logger(), "TF buffer not initialized");
        return false;
    }
    
    try {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf_buffer_->transform(pose, transformed_pose, target_frame_, tf2::durationFromSec(0.1));
        pose = transformed_pose;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Could not transform pose from %s to %s: %s",
                             pose.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return false;
    }
}

neural_network_detector::msg::NeuralNetworkFeedback PersonTrackerProjectionNode::generateFeedback(
    const neural_network_detector::msg::NeuralNetworkDetection& detection)
{
    neural_network_detector::msg::NeuralNetworkFeedback feedback;
    
    // Validate detection bounds
    if (detection.xmax <= detection.xmin || detection.ymax <= detection.ymin) {
        RCLCPP_WARN(this->get_logger(), "Invalid detection bounds for feedback generation");
        return feedback;
    }
    
    // Calculate center and margins
    int16_t center_x = (detection.xmin + detection.xmax) / 2;
    int16_t center_y = (detection.ymin + detection.ymax) / 2;
    
    int16_t height = detection.ymax - detection.ymin;
    
    // Add margins for feedback region
    int16_t margin_y = static_cast<int16_t>(height * feedback_margin_ratio_);
    
    feedback.xcenter = center_x;
    feedback.ymin = std::max(static_cast<int16_t>(0), static_cast<int16_t>(detection.ymin - margin_y));
    feedback.ymax = std::min(static_cast<int16_t>(camera_model_.image_size.height), 
                           static_cast<int16_t>(detection.ymax + margin_y));
    
    // Debug information
    feedback.debug_included = true;
    feedback.head_raw = detection.ymin;
    feedback.feet_raw = detection.ymax;
    feedback.ycenter = center_y;
    
    return feedback;
}

} // namespace person_tracker_projection

// Register the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(person_tracker_projection::PersonTrackerProjectionNode)