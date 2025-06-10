#include "projection_model/person_tracker_projection_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace person_tracker_projection
{

PersonTrackerProjectionNode::PersonTrackerProjectionNode(const rclcpp::NodeOptions & options)
: Node("person_tracker_projection_node", options),
  has_valid_waypoint_(false),
  camera_init_timer_active_(true)
{
    // Initialize parameters
    initializeParameters();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create camera info subscriber first - try different QoS settings
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info",  // Use remapped topic name instead of hardcoded path
        rclcpp::QoS(10).reliable(),  // Use best_effort QoS to match typical camera publishers
        std::bind(&PersonTrackerProjectionNode::cameraInfoCallback, this, std::placeholders::_1));
    
    // Create drone odometry subscriber
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/X3/odom",
        rclcpp::QoS(10),
        std::bind(&PersonTrackerProjectionNode::droneOdomCallback, this, std::placeholders::_1));
    
    // Create publishers
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target_waypoint", rclcpp::QoS(10));
    
    if (publish_feedback_) {
        feedback_pub_ = this->create_publisher<neural_network_msgs::msg::NeuralNetworkFeedback>(
            "feedback", rclcpp::QoS(10));
    }
    
   // In your node constructor or initializer
    camera_init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&PersonTrackerProjectionNode::checkCameraInitialization, this));
    camera_init_timer_active_ = true;

    
    RCLCPP_INFO(this->get_logger(), "Person Tracking Projection Node initialized, waiting for camera info...");
}

void PersonTrackerProjectionNode::checkCameraInitialization()
{
    if (!camera_init_timer_active_) {
        return;
    }
    
    if (camera_model_.initialized) {
        // Camera is initialized, now create the detections subscriber
        RCLCPP_INFO(this->get_logger(), "Camera model initialized! Creating detection subscriber...");
        
        detections_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
            "detections", 
            rclcpp::QoS(10),
            std::bind(&PersonTrackerProjectionNode::detectionsCallback, this, std::placeholders::_1));
        
        // Stop the timer and shutdown camera info subscriber to save resources
        camera_init_timer_->cancel();
        camera_init_timer_ = nullptr;
        camera_init_timer_active_ = false;
        
        // Optionally shutdown camera info subscriber since we don't need continuous updates
        // camera_info_sub_ = nullptr;
        
        RCLCPP_INFO(this->get_logger(), "Node fully initialized and ready for detections!");
    } else {
        // Debug: Check if we're receiving the message at all
        static int debug_counter = 0;
        debug_counter++;
        if (debug_counter % 5 == 0) { // Every 1 second
            RCLCPP_INFO(this->get_logger(), "Still waiting for camera info... (attempt %d)", debug_counter);
            RCLCPP_INFO(this->get_logger(), "Expected topic should be remapped to match: /X3/X3/base_link/camera_front");
        }
    }
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
    this->declare_parameter<double>("min_altitude_threshold", 5.0);
    this->declare_parameter<double>("target_hover_altitude", 7.0);
    this->declare_parameter<bool>("enable_yaw_search", true);
    this->declare_parameter<double>("yaw_search_rate", 0.5);  // rad/s
    this->declare_parameter<double>("detection_timeout", 5.0);  // seconds
    this->declare_parameter<double>("yaw_search_step", 0.2);  // radians per step
    
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
    min_altitude_threshold_ = this->get_parameter("min_altitude_threshold").as_double();
    target_hover_altitude_ = this->get_parameter("target_hover_altitude").as_double();
    enable_yaw_search_ = this->get_parameter("enable_yaw_search").as_bool();
    yaw_search_rate_ = this->get_parameter("yaw_search_rate").as_double();
    detection_timeout_ = this->get_parameter("detection_timeout").as_double();
    yaw_search_step_ = this->get_parameter("yaw_search_step").as_double();
    
    // Initialize yaw search state
    is_searching_ = false;
    search_start_yaw_ = 0.0;
    current_search_yaw_ = 0.0;
    last_detection_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Camera frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Target frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Assumed person height: %.2f m", assumed_person_height_);
    RCLCPP_INFO(this->get_logger(), "  Tracking offset: %.2f m distance, %.2f m height", 
                tracking_offset_distance_, tracking_offset_height_);
    RCLCPP_INFO(this->get_logger(), "  Altitude threshold: %.2f m, hover altitude: %.2f m", 
                min_altitude_threshold_, target_hover_altitude_);
    RCLCPP_INFO(this->get_logger(), "  Yaw search: %s, rate: %.2f rad/s, timeout: %.2f s", 
                enable_yaw_search_ ? "enabled" : "disabled", yaw_search_rate_, detection_timeout_);
}

void PersonTrackerProjectionNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null CameraInfo message");
        return;
    }

    if (camera_model_.initialized) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received camera info: %dx%d", msg->width, msg->height);

    if (msg->k.size() < 9) {
        RCLCPP_ERROR(this->get_logger(), "CameraInfo K matrix has insufficient elements: %zu", msg->k.size());
        return;
    }

    if (msg->width <= 0 || msg->height <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid image dimensions: %dx%d", msg->width, msg->height);
        return;
    }

    if (msg->k[0] <= 0 || msg->k[4] <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid focal lengths: fx=%.3f, fy=%.3f", msg->k[0], msg->k[4]);
        return;
    }

    // ✅ Set up the camera model
    camera_model_.camera_matrix = (cv::Mat_<double>(3, 3) << 
                                    msg->k[0], msg->k[1], msg->k[2],
                                    msg->k[3], msg->k[4], msg->k[5],
                                    msg->k[6], msg->k[7], msg->k[8]);
    camera_model_.distortion_coeffs = cv::Mat(msg->d).clone();
    camera_model_.image_size = cv::Size(msg->width, msg->height);
    camera_model_.initialized = true;

    RCLCPP_INFO(this->get_logger(), "Camera model initialized successfully.");
}


void PersonTrackerProjectionNode::droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null Odometry message");
        return;
    }
    current_odom_ = msg;
}

bool PersonTrackerProjectionNode::checkAndHandleAltitude()
{
    if (!current_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "No drone odometry available for altitude check");
        return false;
    }
    
    double current_altitude = current_odom_->pose.pose.position.z;
    
    if (current_altitude < min_altitude_threshold_) {
        RCLCPP_WARN(this->get_logger(), 
                   "Drone altitude too low (%.2f m < %.2f m). Commanding hover at %.2f m",
                   current_altitude, min_altitude_threshold_, target_hover_altitude_);
        
        // Create hover waypoint at current position but target altitude
        geometry_msgs::msg::PoseStamped hover_waypoint;
        hover_waypoint.header.stamp = this->get_clock()->now();
        hover_waypoint.header.frame_id = target_frame_;
        
        // Keep current X,Y position but set target altitude
        hover_waypoint.pose.position.x = current_odom_->pose.pose.position.x;
        hover_waypoint.pose.position.y = current_odom_->pose.pose.position.y;
        hover_waypoint.pose.position.z = target_hover_altitude_;
        
        // Keep current orientation
        hover_waypoint.pose.orientation = current_odom_->pose.pose.orientation;
        
        // Publish hover command
        if (waypoint_pub_) {
            waypoint_pub_->publish(hover_waypoint);
            RCLCPP_INFO(this->get_logger(), 
                       "Published hover waypoint: [%.2f, %.2f, %.2f]",
                       hover_waypoint.pose.position.x, 
                       hover_waypoint.pose.position.y, 
                       hover_waypoint.pose.position.z);
        }
        
        return false; // Don't proceed with person tracking
    }
    
    return true; // Altitude is sufficient, proceed with tracking
}

void PersonTrackerProjectionNode::handleYawSearch()
{
    if (!enable_yaw_search_ || !current_odom_) {
        return;
    }
    
    auto current_time = this->get_clock()->now();
    double time_since_detection = (current_time - last_detection_time_).seconds();
    
    // Start searching if no detection for timeout period
    if (time_since_detection > detection_timeout_ && !is_searching_) {
        RCLCPP_INFO(this->get_logger(), "No person detected for %.2f seconds, starting yaw search", 
                   time_since_detection);
        startYawSearch();
    }
    
    // Continue searching if already in search mode
    if (is_searching_) {
        performYawSearch();
    }
}

void PersonTrackerProjectionNode::startYawSearch()
{
    if (!current_odom_) {
        return;
    }
    
    is_searching_ = true;
    
    // Get current yaw from quaternion
    tf2::Quaternion q(
        current_odom_->pose.pose.orientation.x,
        current_odom_->pose.pose.orientation.y,
        current_odom_->pose.pose.orientation.z,
        current_odom_->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    search_start_yaw_ = yaw;
    current_search_yaw_ = yaw;
    
    RCLCPP_INFO(this->get_logger(), "Started yaw search from angle: %.2f rad", search_start_yaw_);
}

void PersonTrackerProjectionNode::performYawSearch()
{
    if (!current_odom_) {
        return;
    }
    
    // Increment search yaw
    current_search_yaw_ += yaw_search_step_;
    
    // Check if we've completed a full rotation
    if (std::abs(current_search_yaw_ - search_start_yaw_) >= 2 * M_PI) {
        RCLCPP_INFO(this->get_logger(), "Completed full yaw search rotation, stopping search");
        is_searching_ = false;
        return;
    }
    
    // Create search waypoint with current position but new yaw
    geometry_msgs::msg::PoseStamped search_waypoint;
    search_waypoint.header.stamp = this->get_clock()->now();
    search_waypoint.header.frame_id = target_frame_;
    
    // Keep current position
    search_waypoint.pose.position = current_odom_->pose.pose.position;
    
    // Set new orientation with search yaw
    tf2::Quaternion search_q;
    search_q.setRPY(0, 0, current_search_yaw_);
    search_waypoint.pose.orientation.x = search_q.x();
    search_waypoint.pose.orientation.y = search_q.y();
    search_waypoint.pose.orientation.z = search_q.z();
    search_waypoint.pose.orientation.w = search_q.w();
    
    // Publish search waypoint
    if (waypoint_pub_) {
        waypoint_pub_->publish(search_waypoint);
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published yaw search waypoint: yaw=%.2f rad (%.1f deg)",
                    current_search_yaw_, current_search_yaw_ * 180.0 / M_PI);
    }
}

void PersonTrackerProjectionNode::stopYawSearch()
{
    if (is_searching_) {
        RCLCPP_INFO(this->get_logger(), "Person detected, stopping yaw search");
        is_searching_ = false;
    }
}

void PersonTrackerProjectionNode::detectionsCallback(
    const neural_network_msgs::msg::NeuralNetworkDetectionArray::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null DetectionArray message");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received %zu detections", msg->detections.size());
    
    if (!camera_model_.initialized) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Camera model not initialized yet - this shouldn't happen!");
        return;
    }
    
    // Check altitude before proceeding with detection processing
    if (!checkAndHandleAltitude()) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping detection processing due to insufficient altitude");
        return;
    }
    
    // Handle yaw search for when no person is detected
    handleYawSearch();
    
    // Find the best detection (highest confidence person)
    neural_network_msgs::msg::NeuralNetworkDetection best_detection;
    bool found_person = false;
    double highest_confidence = 0.0;
    
    for (const auto& detection : msg->detections) {
        RCLCPP_DEBUG(this->get_logger(), "Detection: class=%d, confidence=%.3f, bounds=[%d,%d,%d,%d]", 
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
        
        // Filter by class and confidence
        if (detection.object_class == target_person_class_ && 
            detection.detection_score >= min_detection_confidence_ &&
            detection.detection_score > highest_confidence) {
            
            RCLCPP_DEBUG(this->get_logger(), "Found valid person detection with confidence %.3f", 
                        detection.detection_score);
            best_detection = detection;
            highest_confidence = detection.detection_score;
            found_person = true;
        }
    }
    
    if (found_person) {
        RCLCPP_INFO(this->get_logger(), "Processing best detection with confidence %.3f", highest_confidence);
        
        // Stop yaw search since we found a person
        stopYawSearch();
        
        // Update last detection time
        last_detection_time_ = this->get_clock()->now();
        
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
        RCLCPP_DEBUG(this->get_logger(), "No valid person detections found (checked %zu detections)", 
                    msg->detections.size());
    }
}

geometry_msgs::msg::PoseStamped PersonTrackerProjectionNode::projectDetectionToWorld(
    const neural_network_msgs::msg::NeuralNetworkDetection& detection,
    const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = header;
    
    try {
        RCLCPP_DEBUG(this->get_logger(), "Starting projection for detection [%d,%d,%d,%d]",
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
        
        RCLCPP_DEBUG(this->get_logger(), "Foot pixel: [%.1f, %.1f]", foot_pixel.x, foot_pixel.y);
        
        // Convert to normalized image coordinates
        cv::Point2f normalized_point = pixelToNormalizedImageCoordinates(foot_pixel);
        
        RCLCPP_DEBUG(this->get_logger(), "Normalized point: [%.3f, %.3f]", 
                    normalized_point.x, normalized_point.y);
        
        // Get camera height from current drone position
        double camera_height = current_odom_->pose.pose.position.z;
        
        RCLCPP_DEBUG(this->get_logger(), "Camera height: %.2f", camera_height);
        
        // Validate camera height
        if (camera_height <= 0.1) {
            RCLCPP_WARN(this->get_logger(), "Invalid camera height: %.2f", camera_height);
            waypoint.header.frame_id = "";
            return waypoint;
        }
        
        // Project to ground plane
        geometry_msgs::msg::Point ground_point = projectToGround(normalized_point, camera_height);
        
        RCLCPP_DEBUG(this->get_logger(), "Ground point in %s: [%.2f, %.2f, %.2f]", 
                    camera_frame_.c_str(), ground_point.x, ground_point.y, ground_point.z);
        
        // Create pose in camera frame
        waypoint.pose.position = ground_point;
        waypoint.pose.orientation.w = 1.0; // No rotation
        waypoint.header.frame_id = camera_frame_;
        
        // Transform to target frame if needed
        if (transformPoseToTargetFrame(waypoint)) {
            RCLCPP_DEBUG(this->get_logger(), "Transformed to %s: [%.2f, %.2f, %.2f]", 
                        target_frame_.c_str(), 
                        waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
            
            // Add tracking offset (stay behind and above the person)
            waypoint.pose.position.z = std::max(0.0, waypoint.pose.position.z + tracking_offset_height_);
            
            // Calculate offset direction (opposite to person's position relative to drone)
            if (current_odom_) {
                double dx = waypoint.pose.position.x - current_odom_->pose.pose.position.x;
                double dy = waypoint.pose.position.y - current_odom_->pose.pose.position.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                RCLCPP_DEBUG(this->get_logger(), "Drone at [%.2f, %.2f], person at [%.2f, %.2f], distance: %.2f",
                            current_odom_->pose.pose.position.x, current_odom_->pose.pose.position.y,
                            waypoint.pose.position.x, waypoint.pose.position.y, distance);
                
                if (distance > 0.1) { // Avoid division by zero
                    double offset_x = -(dx / distance) * tracking_offset_distance_;
                    double offset_y = -(dy / distance) * tracking_offset_distance_;
                    waypoint.pose.position.x += offset_x;
                    waypoint.pose.position.y += offset_y;
                    
                    RCLCPP_DEBUG(this->get_logger(), "Applied offset [%.2f, %.2f], final waypoint: [%.2f, %.2f, %.2f]",
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

neural_network_msgs::msg::NeuralNetworkFeedback PersonTrackerProjectionNode::generateFeedback(
    const neural_network_msgs::msg::NeuralNetworkDetection& detection)
{
    neural_network_msgs::msg::NeuralNetworkFeedback feedback;
    
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