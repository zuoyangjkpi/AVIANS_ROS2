#include "drone_state_publisher/drone_state_publisher.hpp"
#include <ros2_utils/clock_sync.hpp>

namespace drone_state_publisher {

DroneStatePublisher::DroneStatePublisher() : Node("drone_state_publisher") {
    
    // CRITICAL: Declare use_sim_time parameter FIRST
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", false);
    }
    
    // Log sim time status
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "Using simulation time");
    } else {
        RCLCPP_INFO(this->get_logger(), "Using system time");
    }
    
    // Declare parameters
    this->declare_parameter("optimal_distance", 4.0);
    this->declare_parameter("optimal_height_offset", 2.0);
    this->declare_parameter("optimal_angle_offset", 0.0);  // 0 = directly behind, π/4 = 45° offset
    
    // Get parameters
    optimal_distance_ = this->get_parameter("optimal_distance").as_double();
    optimal_height_offset_ = this->get_parameter("optimal_height_offset").as_double();
    optimal_angle_offset_ = this->get_parameter("optimal_angle_offset").as_double();
    
    // Initialize constant covariance
    initializeConstantCovariance();
    
    // Create subscribers
    drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/X3/odometry", 10,
        std::bind(&DroneStatePublisher::droneOdomCallback, this, std::placeholders::_1));
    
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/machine_1/target_tracker/pose", 10,
        std::bind(&DroneStatePublisher::targetPoseCallback, this, std::placeholders::_1));
    
    target_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/machine_1/target_tracker/twist", 10,
        std::bind(&DroneStatePublisher::targetTwistCallback, this, std::placeholders::_1));
    
    target_offset_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/machine_1/target_tracker/offset", 10,
        std::bind(&DroneStatePublisher::targetOffsetCallback, this, std::placeholders::_1));
    
    // Create publishers
    uav_pose_pub_ = this->create_publisher<uav_msgs::msg::UAVPose>("/machine_1/pose", 10);
    uav_raw_pose_pub_ = this->create_publisher<uav_msgs::msg::UAVPose>("/machine_1/pose/raw", 10);
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_waypoint", 10);
    
    RCLCPP_INFO(this->get_logger(), "Drone State Publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Optimal viewing - Distance: %.1fm, Height offset: %.1fm, Angle: %.1f°", 
                optimal_distance_, optimal_height_offset_, optimal_angle_offset_ * 180.0 / M_PI);
}

void DroneStatePublisher::initializeConstantCovariance() {
    // Initialize with reasonable constant values for UAV pose uncertainty
    // 10x10 covariance matrix: pos(3), vel(3), quat(4)
    std::fill(constant_covariance_.begin(), constant_covariance_.end(), 0.0);
    
    // Position uncertainties (m^2)
    constant_covariance_[0] = 0.01;   // x position variance
    constant_covariance_[11] = 0.01;  // y position variance  
    constant_covariance_[22] = 0.01;  // z position variance
    
    // Velocity uncertainties (m^2/s^2)
    constant_covariance_[33] = 0.005; // x velocity variance
    constant_covariance_[44] = 0.005; // y velocity variance
    constant_covariance_[55] = 0.005; // z velocity variance
    
    // Quaternion uncertainties
    constant_covariance_[66] = 0.001; // qw variance
    constant_covariance_[77] = 0.001; // qx variance
    constant_covariance_[88] = 0.001; // qy variance
    constant_covariance_[99] = 0.001; // qz variance
}

bool DroneStatePublisher::validateTimestamp(const rclcpp::Time& timestamp)  {
    // Use const_cast to work with the shared_ptr requirement
    auto drone_ptr = std::static_pointer_cast<DroneStatePublisher>(shared_from_this());
    return ros2_utils::ClockSynchronizer::validateTimestamp(drone_ptr, timestamp);
}

void DroneStatePublisher::droneOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Validate timestamp
    if (!validateTimestamp(rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in odometry message");
        return;
    }
    
    current_drone_odom_ = msg;
    
    // Publish BOTH UAV pose topics whenever we get new odometry
    publishRawUAVPose();  // Raw pose without offset correction
    publishUAVPose();     // Pose with offset correction (if available)
    
    // Publish waypoint if we have target information
    if (current_target_pose_) {
        publishTargetWaypoint();
    }
}

void DroneStatePublisher::targetPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Validate timestamp
    if (!validateTimestamp(rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in target pose message");
        return;
    }
    
    current_target_pose_ = msg;
    
    // Publish waypoint if we have drone odometry
    if (current_drone_odom_) {
        publishTargetWaypoint();
    }
}

void DroneStatePublisher::targetTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    // Validate timestamp
    if (!validateTimestamp(rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in target twist message");
        return;
    }
    
    current_target_twist_ = msg;
}

void DroneStatePublisher::targetOffsetCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Validate timestamp
    if (!validateTimestamp(rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in target offset message");
        return;
    }
    
    current_target_offset_ = msg;
}

void DroneStatePublisher::publishRawUAVPose() {
    if (!current_drone_odom_) {
        return;
    }
    
    auto uav_pose = uav_msgs::msg::UAVPose();
    
    // Copy header
    uav_pose.header = current_drone_odom_->header;
    
    // Copy position (RAW - no offset correction)
    uav_pose.position = current_drone_odom_->pose.pose.position;
    
    // Copy velocity (convert Vector3 to Point)
    uav_pose.velocity.x = current_drone_odom_->twist.twist.linear.x;
    uav_pose.velocity.y = current_drone_odom_->twist.twist.linear.y;
    uav_pose.velocity.z = current_drone_odom_->twist.twist.linear.z;
    
    // Copy orientation
    uav_pose.orientation = current_drone_odom_->pose.pose.orientation;
    
    // Angular velocity (convert Vector3 to Point)
    uav_pose.ang_velocity.x = current_drone_odom_->twist.twist.angular.x;
    uav_pose.ang_velocity.y = current_drone_odom_->twist.twist.angular.y;
    uav_pose.ang_velocity.z = current_drone_odom_->twist.twist.angular.z;
    
    // Use constant covariance
    uav_pose.covariance = constant_covariance_;
    
    // Default values for flight parameters
    uav_pose.thrust = 0.5;  // Neutral thrust
    uav_pose.flightmode = 0; // Default flight mode
    
    // POI - could be target position if available
    if (current_target_pose_) {
        uav_pose.poi = current_target_pose_->pose.pose.position;
    } else {
        uav_pose.poi.x = 0.0;
        uav_pose.poi.y = 0.0;
        uav_pose.poi.z = 0.0;
    }
    
    uav_pose.header.frame_id = "world";
    uav_raw_pose_pub_->publish(uav_pose);
}

void DroneStatePublisher::publishUAVPose() {
    if (!current_drone_odom_) {
        return;
    }
    
    auto uav_pose = uav_msgs::msg::UAVPose();
    
    // Copy header
    uav_pose.header = current_drone_odom_->header;
    
    // Copy position and APPLY OFFSET CORRECTION if available
    uav_pose.position = current_drone_odom_->pose.pose.position;
    
    // Apply Kalman filter offset correction if available
    if (current_target_offset_) {
        uav_pose.position.x += current_target_offset_->pose.pose.position.x;
        uav_pose.position.y += current_target_offset_->pose.pose.position.y;
        uav_pose.position.z += current_target_offset_->pose.pose.position.z;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Applied offset correction: [%.3f, %.3f, %.3f]",
                    current_target_offset_->pose.pose.position.x,
                    current_target_offset_->pose.pose.position.y,
                    current_target_offset_->pose.pose.position.z);
    }
    
    // Copy velocity (convert Vector3 to Point)
    uav_pose.velocity.x = current_drone_odom_->twist.twist.linear.x;
    uav_pose.velocity.y = current_drone_odom_->twist.twist.linear.y;
    uav_pose.velocity.z = current_drone_odom_->twist.twist.linear.z;
    
    // Copy orientation
    uav_pose.orientation = current_drone_odom_->pose.pose.orientation;
    
    // Angular velocity (convert Vector3 to Point)
    uav_pose.ang_velocity.x = current_drone_odom_->twist.twist.angular.x;
    uav_pose.ang_velocity.y = current_drone_odom_->twist.twist.angular.y;
    uav_pose.ang_velocity.z = current_drone_odom_->twist.twist.angular.z;
    
    // Use constant covariance
    uav_pose.covariance = constant_covariance_;
    
    // Default values for flight parameters
    uav_pose.thrust = 0.5;  // Neutral thrust
    uav_pose.flightmode = 0; // Default flight mode
    
    // Point of interest (POI) - could be target position if available
    if (current_target_pose_) {
        uav_pose.poi = current_target_pose_->pose.pose.position;
    } else {
        uav_pose.poi.x = 0.0;
        uav_pose.poi.y = 0.0;
        uav_pose.poi.z = 0.0;
    }
    
    uav_pose.header.frame_id = "world";
    uav_pose_pub_->publish(uav_pose);
}

void DroneStatePublisher::publishTargetWaypoint() {
    if (!current_target_pose_ || !current_drone_odom_) {
        return;
    }
    
    auto waypoint = calculateOptimalViewingPose();
    waypoint_pub_->publish(waypoint);
}

geometry_msgs::msg::PoseStamped DroneStatePublisher::calculateOptimalViewingPose() {
    auto waypoint = geometry_msgs::msg::PoseStamped();
    waypoint.header.stamp = ros2_utils::ClockSynchronizer::getSafeTime(shared_from_this());
    waypoint.header.frame_id = "world";
    
    // Target position
    double target_x = current_target_pose_->pose.pose.position.x;
    double target_y = current_target_pose_->pose.pose.position.y;
    double target_z = current_target_pose_->pose.pose.position.z;
    
    // Calculate target velocity direction for predictive positioning
    double vel_x = 0.0, vel_y = 0.0;
    if (current_target_twist_) {
        vel_x = current_target_twist_->twist.twist.linear.x;
        vel_y = current_target_twist_->twist.twist.linear.y;
    }
    
    // Calculate desired camera viewing angle
    double target_heading = 0.0;
    if (sqrt(vel_x * vel_x + vel_y * vel_y) > 0.1) {
        // Use velocity direction if target is moving
        target_heading = atan2(vel_y, vel_x);
    } else {
        // If target is stationary, maintain current relative position
        double drone_x = current_drone_odom_->pose.pose.position.x;
        double drone_y = current_drone_odom_->pose.pose.position.y;
        target_heading = atan2(drone_y - target_y, drone_x - target_x);
    }
    
    // Add the angle offset for optimal viewing
    double viewing_angle = target_heading + M_PI + optimal_angle_offset_;
    
    // Calculate optimal drone position
    waypoint.pose.position.x = target_x + optimal_distance_ * cos(viewing_angle);
    waypoint.pose.position.y = target_y + optimal_distance_ * sin(viewing_angle);
    waypoint.pose.position.z = target_z + optimal_height_offset_;
    
    // Calculate drone orientation to look at target
    double look_at_yaw = atan2(target_y - waypoint.pose.position.y, 
                               target_x - waypoint.pose.position.x);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, look_at_yaw);
    waypoint.pose.orientation = tf2::toMsg(q);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Target: [%.2f, %.2f, %.2f] -> Waypoint: [%.2f, %.2f, %.2f], Yaw: %.2f°",
                target_x, target_y, target_z,
                waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z,
                look_at_yaw * 180.0 / M_PI);
    
    return waypoint;
}

} // namespace drone_state_publisher

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<drone_state_publisher::DroneStatePublisher>();
    
    WAIT_FOR_CLOCK_DELAYED(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting Drone State Publisher Node...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
