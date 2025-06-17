//
// tf_from_uav_pose_ros2.cpp
// ROS2 migration of tf_from_uav_poses_node
// Migrated by Assistant on 16.06.25
//

#include <tf_from_uav_pose/tf_from_uav_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ros2_utils/clock_sync.hpp>

namespace tf_from_uav_pose {

TfFromUAVPose::TfFromUAVPose() : Node("tf_from_uav_pose") {
    
    // CRITICAL: Declare use_sim_time parameter FIRST
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", true);
    }
    
    // Initialize parameters first
    initializeParameters();
    
    // Initialize TF broadcasters
    if (!dont_publish_tfs_) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' and publishing tf with parent '%s' and child '%s'", 
                   pose_topic_name_.c_str(), world_frame_id_.c_str(), machine_frame_id_.c_str());
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    } else {
        RCLCPP_INFO(this->get_logger(), "Requested to not publish TFs - Publishing only poses!");
    }

    // Set up parameter callback for dynamic reconfigure equivalent
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TfFromUAVPose::parametersCallback, this, std::placeholders::_1));

    // Setup static transforms (with placeholder timestamps - will be updated after clock sync)
    setupStaticTransforms();
    
    // Initialize publishers and subscribers
    initializePublishers();
    initializeSubscribers();

    RCLCPP_INFO(this->get_logger(), "TF from UAV Pose ROS2 node initialized successfully");
}

void TfFromUAVPose::initializeParameters() {
    // Get sim time status for logging
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "Using simulation time");
    } else {
        RCLCPP_INFO(this->get_logger(), "Using system time");
    }
    
    // Declare topic name parameters
    this->declare_parameter("pose_topic_name", pose_topic_name_);
    this->declare_parameter("raw_pose_topic_name", raw_pose_topic_name_);
    this->declare_parameter("std_pose_topic_name", std_pose_topic_name_);
    this->declare_parameter("std_raw_pose_topic_name", std_raw_pose_topic_name_);
    this->declare_parameter("throttled_pose_topic_name", throttled_pose_topic_name_);
    this->declare_parameter("throttled_uav_pose_topic_name", throttled_uav_pose_topic_name_);
    
    // Declare frame ID parameters
    this->declare_parameter("machine_frame_id", machine_frame_id_);
    this->declare_parameter("world_frame_id", world_frame_id_);
    this->declare_parameter("world_enu_frame_id", world_enu_frame_id_);
    this->declare_parameter("world_nwu_frame_id", world_nwu_frame_id_);
    this->declare_parameter("camera_frame_id", camera_frame_id_);
    this->declare_parameter("camera_rgb_optical_frame_id", camera_optical_frame_id_);
    
    // Declare offset and covariance parameters
    this->declare_parameter("offset_x", offset_[0]);
    this->declare_parameter("offset_y", offset_[1]);
    this->declare_parameter("offset_z", offset_[2]);
    this->declare_parameter("covariance_x", added_covariance_[0]);
    this->declare_parameter("covariance_y", added_covariance_[1]);
    this->declare_parameter("covariance_z", added_covariance_[2]);
    
    // Other parameters
    this->declare_parameter("throttle_rate", throttle_rate_);
    this->declare_parameter("dont_publish_tfs", dont_publish_tfs_);
    
    // Camera static publishing parameters
    this->declare_parameter("camera_static_publish.publish", publish_camera_to_robot_tf_and_pose_);
    this->declare_parameter("camera_static_publish.tf_parameters", camera_tf_parameters_);
    this->declare_parameter("camera_static_publish.topic", std::string("/machine_1/camera/pose"));
    this->declare_parameter("camera_static_publish.pose_optical_topic", std::string("/machine_1/camera/pose_optical"));

    // Get parameter values
    pose_topic_name_ = this->get_parameter("pose_topic_name").as_string();
    raw_pose_topic_name_ = this->get_parameter("raw_pose_topic_name").as_string();
    std_pose_topic_name_ = this->get_parameter("std_pose_topic_name").as_string();
    std_raw_pose_topic_name_ = this->get_parameter("std_raw_pose_topic_name").as_string();
    throttled_pose_topic_name_ = this->get_parameter("throttled_pose_topic_name").as_string();
    throttled_uav_pose_topic_name_ = this->get_parameter("throttled_uav_pose_topic_name").as_string();
    
    machine_frame_id_ = this->get_parameter("machine_frame_id").as_string();
    world_frame_id_ = this->get_parameter("world_frame_id").as_string();
    world_enu_frame_id_ = this->get_parameter("world_enu_frame_id").as_string();
    world_nwu_frame_id_ = this->get_parameter("world_nwu_frame_id").as_string();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    camera_optical_frame_id_ = this->get_parameter("camera_rgb_optical_frame_id").as_string();
    
    offset_[0] = this->get_parameter("offset_x").as_double();
    offset_[1] = this->get_parameter("offset_y").as_double();
    offset_[2] = this->get_parameter("offset_z").as_double();
    added_covariance_[0] = this->get_parameter("covariance_x").as_double();
    added_covariance_[1] = this->get_parameter("covariance_y").as_double();
    added_covariance_[2] = this->get_parameter("covariance_z").as_double();
    
    throttle_rate_ = this->get_parameter("throttle_rate").as_double();
    dont_publish_tfs_ = this->get_parameter("dont_publish_tfs").as_bool();
    
    publish_camera_to_robot_tf_and_pose_ = this->get_parameter("camera_static_publish.publish").as_bool();
    camera_tf_parameters_ = this->get_parameter("camera_static_publish.tf_parameters").as_double_array();

    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Offset: [%.3f, %.3f, %.3f]", offset_[0], offset_[1], offset_[2]);
    RCLCPP_INFO(this->get_logger(), "  Added covariance: [%.3f, %.3f, %.3f]", 
               added_covariance_[0], added_covariance_[1], added_covariance_[2]);
    RCLCPP_INFO(this->get_logger(), "  Throttle rate: %.1f Hz", throttle_rate_);
}

void TfFromUAVPose::setupStaticTransforms() {
    // Prepare TF Messages with placeholder timestamps (will be updated after clock sync)
    rclcpp::Time placeholder_time(0);
    
    // TF from world to machine (this will be dynamic)
    tf_pose_.header.frame_id = world_frame_id_;
    tf_pose_.child_frame_id = machine_frame_id_;

    // TF from world_ENU to world (world_ENU will be the root of the tree)
    tf_world_enu_.header.stamp = placeholder_time;
    tf_world_enu_.header.frame_id = world_enu_frame_id_;
    tf_world_enu_.child_frame_id = world_frame_id_;
    tf2::Quaternion q_enu;
    q_enu.setEuler(M_PI, 0, M_PI_2);
    q_enu.normalize();
    tf_world_enu_.transform.rotation = tf2::toMsg(q_enu);

    // TF from world to world_NWU
    tf_world_nwu_.header.stamp = placeholder_time;
    tf_world_nwu_.header.frame_id = world_frame_id_;
    tf_world_nwu_.child_frame_id = world_nwu_frame_id_;
    tf2::Quaternion q_nwu;
    q_nwu.setEuler(0, M_PI, 0);
    tf_world_nwu_.transform.rotation = tf2::toMsg(q_nwu);

    // TF from camera to rgb optical link
    tf_cam_rgb_.header.stamp = placeholder_time;
    tf_cam_rgb_.header.frame_id = camera_frame_id_;
    tf_cam_rgb_.child_frame_id = camera_optical_frame_id_;
    tf2::Quaternion q_cr;
    q_cr.setEuler(M_PI_2, 0, M_PI_2);
    tf_cam_rgb_.transform.rotation = tf2::toMsg(q_cr);

    // Setup camera transforms if requested
    if (publish_camera_to_robot_tf_and_pose_) {
        setupCameraTransforms();
    }

    RCLCPP_INFO(this->get_logger(), "Static transforms configured (timestamps will be updated after clock sync)");
}

void TfFromUAVPose::setupCameraTransforms() {
    if (camera_tf_parameters_.size() < 7) {
        RCLCPP_WARN(this->get_logger(), "Camera TF parameters incomplete, need 7 values (x,y,z,qx,qy,qz,qw)");
        return;
    }

    // Camera transform with placeholder timestamp
    camera_transform_.header.frame_id = machine_frame_id_;
    camera_transform_.child_frame_id = camera_frame_id_;
    camera_transform_.header.stamp = rclcpp::Time(0);  // Will be updated after clock sync

    // Set translation
    camera_transform_.transform.translation.x = camera_tf_parameters_[0];
    camera_transform_.transform.translation.y = camera_tf_parameters_[1];
    camera_transform_.transform.translation.z = camera_tf_parameters_[2];

    // Set rotation
    camera_transform_.transform.rotation.x = camera_tf_parameters_[3];
    camera_transform_.transform.rotation.y = camera_tf_parameters_[4];
    camera_transform_.transform.rotation.z = camera_tf_parameters_[5];
    camera_transform_.transform.rotation.w = camera_tf_parameters_[6];

    // Setup camera pose message with placeholder timestamp
    cam_rob_pose_.header = camera_transform_.header;
    cam_rob_pose_.header.stamp = rclcpp::Time(0);  // Will be updated
    cam_rob_pose_.pose.pose.position.x = camera_transform_.transform.translation.x;
    cam_rob_pose_.pose.pose.position.y = camera_transform_.transform.translation.y;
    cam_rob_pose_.pose.pose.position.z = camera_transform_.transform.translation.z;
    cam_rob_pose_.pose.pose.orientation = camera_transform_.transform.rotation;

    // Setup RGB camera pose
    rgb_cam_pose_.header.frame_id = camera_frame_id_;
    rgb_cam_pose_.header.stamp = rclcpp::Time(0);  // Will be updated
    rgb_cam_pose_.pose.pose.position.x = 0.0;
    rgb_cam_pose_.pose.pose.position.y = 0.0;
    rgb_cam_pose_.pose.pose.position.z = 0.0;
    rgb_cam_pose_.pose.pose.orientation = tf_cam_rgb_.transform.rotation;

    RCLCPP_INFO(this->get_logger(), "Camera to robot transform and poses configured");
}

void TfFromUAVPose::updateTimestampsAfterClockSync() {
    auto current_time = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Updating static transform timestamps after clock sync: %.3f", 
               current_time.seconds());
    
    // Update all static transform timestamps
    tf_world_enu_.header.stamp = current_time;
    tf_world_nwu_.header.stamp = current_time; 
    tf_cam_rgb_.header.stamp = current_time;
    
    if (publish_camera_to_robot_tf_and_pose_) {
        camera_transform_.header.stamp = current_time;
        cam_rob_pose_.header.stamp = current_time;
        rgb_cam_pose_.header.stamp = current_time;
    }
    
    // Re-broadcast static transforms with correct timestamps
    if (!dont_publish_tfs_ && static_tf_broadcaster_) {
        std::vector<geometry_msgs::msg::TransformStamped> static_tfs{
            tf_world_nwu_, tf_world_enu_, tf_cam_rgb_
        };
        if (publish_camera_to_robot_tf_and_pose_) {
            static_tfs.push_back(camera_transform_);
        }
        static_tf_broadcaster_->sendTransform(static_tfs);
        RCLCPP_INFO(this->get_logger(), "Re-broadcasted %zu static transforms with synced timestamps", 
                   static_tfs.size());
    }
}

void TfFromUAVPose::initializePublishers() {
    // Standard pose publishers
    std_pose_.header.frame_id = world_frame_id_;
    std_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        std_pose_topic_name_, 10);

    std_raw_pose_.header.frame_id = world_frame_id_;
    std_raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        std_raw_pose_topic_name_, 10);

    // Throttled pose publishers
    throttled_pose_.header.frame_id = world_frame_id_;
    throttled_pose_.header.stamp = rclcpp::Time(0);  // Will be updated
    throttled_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        throttled_pose_topic_name_, 10);
    throttled_uav_pose_pub_ = this->create_publisher<uav_msgs::msg::UAVPose>(
        throttled_uav_pose_topic_name_, 10);

    // Camera pose publishers if enabled
   if (publish_camera_to_robot_tf_and_pose_) {
        std::string camera_pose_topic = this->get_parameter("camera_static_publish.topic").as_string();
        std::string cam_rgb_pose_topic = this->get_parameter("camera_static_publish.pose_optical_topic").as_string();
        
        camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            camera_pose_topic, rclcpp::QoS(1).transient_local());
        cam_rgb_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            cam_rgb_pose_topic, rclcpp::QoS(1).transient_local());

        // Timer to continuously publish camera poses with updated timestamps
        camera_pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            [this]() {
                auto current_time = ros2_utils::ClockSynchronizer::getSafeTime(shared_from_this());
                if (current_time.nanoseconds() > 0) {
                    if (camera_pose_pub_) {
                        cam_rob_pose_.header.stamp = current_time;
                        camera_pose_pub_->publish(cam_rob_pose_);
                    }
                    if (cam_rgb_pose_pub_) {
                        rgb_cam_pose_.header.stamp = current_time;
                        cam_rgb_pose_pub_->publish(rgb_cam_pose_);
                    }
                }
            });
            
        RCLCPP_INFO(this->get_logger(), "Camera pose publishers and timer configured");
    }

    RCLCPP_INFO(this->get_logger(), "Publishers initialized");
}

void TfFromUAVPose::initializeSubscribers() {
    // Subscribe to poses
    pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        pose_topic_name_, 10, 
        std::bind(&TfFromUAVPose::poseCallback, this, std::placeholders::_1));

    raw_pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        raw_pose_topic_name_, 10,
        std::bind(&TfFromUAVPose::rawPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers initialized");
    RCLCPP_INFO(this->get_logger(), "  Listening to: %s", pose_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Listening to: %s", raw_pose_topic_name_.c_str());
}

void TfFromUAVPose::poseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    // Validate timestamp
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in pose message");
        return;
    }
    
    // Copy contents to std pose msg
    std_pose_.header.stamp = msg->header.stamp;
    std_pose_.pose.pose.position = msg->position;
    std_pose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, std_pose_.pose);

    // Add offset
    try {
        std_pose_.pose.pose.position.x += offset_[0];
        std_pose_.pose.pose.position.y += offset_[1];
        std_pose_.pose.pose.position.z += offset_[2];

        std_pose_.pose.covariance[0] += added_covariance_[0];
        std_pose_.pose.covariance[7] += added_covariance_[1];
        std_pose_.pose.covariance[14] += added_covariance_[2];
    } catch (const std::out_of_range &oor) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't add offset: " << oor.what());
        return;
    }

    // Publish std pose msg
    std_pose_pub_->publish(std_pose_);

    // Handle throttled publishing
    rclcpp::Duration time_diff = rclcpp::Time(msg->header.stamp) - rclcpp::Time(throttled_pose_.header.stamp);
    if (time_diff.seconds() > 0 && (1.0 / time_diff.seconds()) <= throttle_rate_) {
        // Copy contents to throttle pose msg
        throttled_pose_.header.stamp = msg->header.stamp;
        throttled_pose_.pose.position = msg->position;
        throttled_pose_.pose.orientation = msg->orientation;

        // Publish throttle pose msg
        throttled_pose_pub_->publish(throttled_pose_);
        throttled_uav_pose_pub_->publish(*msg);
    }

    // Copy contents to tf msgs
    tf_pose_.header.stamp = msg->header.stamp;

    tf_pose_.transform.translation.x = std_pose_.pose.pose.position.x;
    tf_pose_.transform.translation.y = std_pose_.pose.pose.position.y;
    tf_pose_.transform.translation.z = std_pose_.pose.pose.position.z;

    tf_pose_.transform.rotation.w = std_pose_.pose.pose.orientation.w;
    tf_pose_.transform.rotation.x = std_pose_.pose.pose.orientation.x;
    tf_pose_.transform.rotation.y = std_pose_.pose.pose.orientation.y;
    tf_pose_.transform.rotation.z = std_pose_.pose.pose.orientation.z;

    // Broadcast tf
    if (!dont_publish_tfs_ && tf_broadcaster_) {
        tf_broadcaster_->sendTransform(tf_pose_);
    }
}

void TfFromUAVPose::rawPoseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {
    // Validate timestamp
    if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp in raw pose message");
        return;
    }
    
    // Copy contents to std pose msg
    std_raw_pose_.header.stamp = msg->header.stamp;
    std_raw_pose_.pose.pose.position = msg->position;
    std_raw_pose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, std_raw_pose_.pose);

    // Publish std pose msg
    std_raw_pose_pub_->publish(std_raw_pose_);
}

rcl_interfaces::msg::SetParametersResult TfFromUAVPose::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    RCLCPP_INFO(this->get_logger(), "Received parameter update request");

    for (const auto & param : parameters) {
        if (param.get_name() == "offset_x") {
            offset_[0] = param.as_double();
        } else if (param.get_name() == "offset_y") {
            offset_[1] = param.as_double();
        } else if (param.get_name() == "offset_z") {
            offset_[2] = param.as_double();
        } else if (param.get_name() == "covariance_x") {
            added_covariance_[0] = param.as_double();
        } else if (param.get_name() == "covariance_y") {
            added_covariance_[1] = param.as_double();
        } else if (param.get_name() == "covariance_z") {
            added_covariance_[2] = param.as_double();
        } else if (param.get_name() == "throttle_rate") {
            throttle_rate_ = param.as_double();
        } else if (param.get_name() == "use_sim_time") {
            bool use_sim_time = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "use_sim_time updated to: %s", use_sim_time ? "true" : "false");
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Throttle Rate: " << throttle_rate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Offset: [" << offset_[0] << ", " << offset_[1] << ", " << offset_[2] << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Extra covariance: [" << added_covariance_[0] << ", " << added_covariance_[1] << ", " << added_covariance_[2] << "]");

    return result;
}

void uavCovariance_to_rosCovariance(const uav_msgs::msg::UAVPose::SharedPtr uav_msg,
                                    geometry_msgs::msg::PoseWithCovariance &std_pose_cov) {
    
    // MRPT-free implementation: Direct covariance mapping
    // This avoids all MRPT compatibility issues
    
    // Initialize covariance matrix to zero
    std::fill(std_pose_cov.covariance.begin(), std_pose_cov.covariance.end(), 0.0);
    
    // UAV covariance matrix format (10x10):
    // [0-2]: position (north, east, down) = (x, y, z)
    // [3-5]: velocity  
    // [6-9]: quaternion (w, x, y, z)
    
    // Map position covariances directly (assuming same coordinate frame)
    if (uav_msg->covariance.size() >= 100) { // 10x10 matrix
        // Position block (3x3) - indices 0,1,2 in UAV = 0,1,2 in ROS
        std_pose_cov.covariance[0]  = uav_msg->covariance[0];   // x-x (north-north)
        std_pose_cov.covariance[1]  = uav_msg->covariance[1];   // x-y (north-east)  
        std_pose_cov.covariance[2]  = uav_msg->covariance[2];   // x-z (north-down)
        std_pose_cov.covariance[6]  = uav_msg->covariance[10];  // y-x (east-north)
        std_pose_cov.covariance[7]  = uav_msg->covariance[11];  // y-y (east-east)
        std_pose_cov.covariance[8]  = uav_msg->covariance[12];  // y-z (east-down)
        std_pose_cov.covariance[12] = uav_msg->covariance[20];  // z-x (down-north)
        std_pose_cov.covariance[13] = uav_msg->covariance[21];  // z-y (down-east)
        std_pose_cov.covariance[14] = uav_msg->covariance[22];  // z-z (down-down)
        
        // For orientation: approximate from quaternion covariances
        // This is a simplification - proper conversion would require Jacobians
        double qw_var = uav_msg->covariance[66]; // q_w variance
        double qx_var = uav_msg->covariance[77]; // q_x variance  
        double qy_var = uav_msg->covariance[88]; // q_y variance
        double qz_var = uav_msg->covariance[99]; // q_z variance
        
        // Approximate Euler angle variances from quaternion variances
        // This is a rough approximation: var(euler) ≈ 4 * var(quat)
        std_pose_cov.covariance[21] = 4.0 * (qx_var + qw_var); // roll variance
        std_pose_cov.covariance[28] = 4.0 * (qy_var + qw_var); // pitch variance
        std_pose_cov.covariance[35] = 4.0 * (qz_var + qw_var); // yaw variance
        
    } else {
        // Fallback: use reasonable default uncertainties
        RCLCPP_WARN_ONCE(rclcpp::get_logger("tf_from_uav_pose"), 
                         "UAV covariance array too small, using default uncertainties");
        
        // Default position uncertainties (m^2)
        std_pose_cov.covariance[0]  = 0.1;  // x variance
        std_pose_cov.covariance[7]  = 0.1;  // y variance  
        std_pose_cov.covariance[14] = 0.1;  // z variance
        
        // Default orientation uncertainties (rad^2)
        std_pose_cov.covariance[21] = 0.01; // roll variance
        std_pose_cov.covariance[28] = 0.01; // pitch variance
        std_pose_cov.covariance[35] = 0.01; // yaw variance
    }
}

} // namespace tf_from_uav_pose

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<tf_from_uav_pose::TfFromUAVPose>();
    
    WAIT_FOR_CLOCK_DELAYED(node);
    
    // CRITICAL: Update timestamps after clock sync
    node->updateTimestampsAfterClockSync();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}