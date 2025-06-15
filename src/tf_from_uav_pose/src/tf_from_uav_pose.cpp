//
// tf_from_uav_poses_node.cpp
// C++17 is required for ROS2. Deal with it!
//

#include "tf_from_uav_pose/tf_from_uav_pose.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//#include <mrpt/poses.h>
//#include <mrpt/math.h>
// #include <mrpt/poses/CPose3DQuatPDFGaussian.h>
// #include <mrpt/poses/CPose3DPDFGaussian.h>
// #include <mrpt/ros2bridge/pose.h>
#include <Eigen/Geometry> 

namespace tf_from_uav_pose {

TfFromUAVPose::TfFromUAVPose() : Node("tf_from_uav_pose") {

    std::string
            pose_topic_name{"pose"},
            raw_pose_topic_name{"pose/raw"},
            std_pose_topic_name{"pose/corr/std"},
            std_raw_pose_topic_name{"pose/raww/std"},
            throttled_pose_topic_name{"throttledPose"},
            throttled_uav_pose_topic_name{"throttledUAVPose"},
            machine_frame_id{"machine_1_base_link"},
            world_frame_id{"world"},
            world_enu_frame_id{"world_ENU"},
            world_nwu_frame_id{"world_NWU"},
            camera_frame_id{"cameraFrameID"},
            camera_rgb_optical_frame_id{"cameraRGBOpticalFrameID"};

    // Declare parameters with default values
    this->declare_parameter("poseTopicName", pose_topic_name);
    this->declare_parameter("rawPoseTopicName", raw_pose_topic_name);
    this->declare_parameter("stdPoseTopicName", std_pose_topic_name);
    this->declare_parameter("stdRawPoseTopicName", std_raw_pose_topic_name);
    this->declare_parameter("throttledPoseTopicName", throttled_pose_topic_name);
    this->declare_parameter("throttledUAVPoseTopicName", throttled_uav_pose_topic_name);
    this->declare_parameter("machineFrameID", machine_frame_id);
    this->declare_parameter("worldFrameID", world_frame_id);
    this->declare_parameter("worldENUFrameID", world_enu_frame_id);
    this->declare_parameter("worldNWUFrameID", world_nwu_frame_id);
    this->declare_parameter("cameraFrameID", camera_frame_id);
    this->declare_parameter("cameraRGBOpticalFrameID", camera_rgb_optical_frame_id);
    this->declare_parameter("dontPublishTFs", dont_publish_tfs_);

    // Get parameter values
    this->get_parameter("poseTopicName", pose_topic_name);
    this->get_parameter("rawPoseTopicName", raw_pose_topic_name);
    this->get_parameter("stdPoseTopicName", std_pose_topic_name);
    this->get_parameter("stdRawPoseTopicName", std_raw_pose_topic_name);
    this->get_parameter("throttledPoseTopicName", throttled_pose_topic_name);
    this->get_parameter("throttledUAVPoseTopicName", throttled_uav_pose_topic_name);
    this->get_parameter("machineFrameID", machine_frame_id);
    this->get_parameter("worldFrameID", world_frame_id);
    this->get_parameter("worldENUFrameID", world_enu_frame_id);
    this->get_parameter("worldNWUFrameID", world_nwu_frame_id);
    this->get_parameter("cameraFrameID", camera_frame_id);
    this->get_parameter("cameraRGBOpticalFrameID", camera_rgb_optical_frame_id);
    this->get_parameter("dontPublishTFs", dont_publish_tfs_);

    // Declare dynamic reconfigure equivalent parameters
    this->declare_parameter("offsetX", offset_[0]);
    this->declare_parameter("offsetY", offset_[1]);
    this->declare_parameter("offsetZ", offset_[2]);
    this->declare_parameter("covarianceX", added_covariance_[0]);
    this->declare_parameter("covarianceY", added_covariance_[1]);
    this->declare_parameter("covarianceZ", added_covariance_[2]);
    this->declare_parameter("throttleRate", throttle_rate_);

    // Get dynamic parameter values
    this->get_parameter("offsetX", offset_[0]);
    this->get_parameter("offsetY", offset_[1]);
    this->get_parameter("offsetZ", offset_[2]);
    this->get_parameter("covarianceX", added_covariance_[0]);
    this->get_parameter("covarianceY", added_covariance_[1]);
    this->get_parameter("covarianceZ", added_covariance_[2]);
    this->get_parameter("throttleRate", throttle_rate_);

    // Set up parameter callback for dynamic reconfigure equivalent
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TfFromUAVPose::parametersCallback, this, std::placeholders::_1));

    if (!dont_publish_tfs_) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' and publishing tf with parent '%s' and child '%s'", 
                   pose_topic_name.c_str(), world_frame_id.c_str(), machine_frame_id.c_str());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    } else {
        RCLCPP_INFO(this->get_logger(), "Requested to not publish TFs - Publishing only poses!");
    }

    // Prepare TF Messages
    // TF from world to machine
    tf_pose_.header.frame_id = world_frame_id;
    tf_pose_.child_frame_id = machine_frame_id;

    // TF from world_ENU to world (world_ENU will be the root of the tree)
    tf_world_enu_.header.stamp = this->now();
    tf_world_enu_.header.frame_id = world_enu_frame_id;
    tf_world_enu_.child_frame_id = world_frame_id;
    tf2::Quaternion q_enu;
    q_enu.setEuler(M_PI, 0, M_PI_2);
    q_enu.normalize();
    tf_world_enu_.transform.rotation = tf2::toMsg(q_enu);

    // TF from world to world_NWU
    tf_world_nwu_.header.stamp = tf_world_enu_.header.stamp;
    tf_world_nwu_.header.frame_id = world_frame_id;
    tf_world_nwu_.child_frame_id = world_nwu_frame_id;
    tf2::Quaternion q_nwu;
    q_nwu.setEuler(0, M_PI, 0);
    tf_world_nwu_.transform.rotation = tf2::toMsg(q_nwu);

    // TF from camera to rgb optical link
    tf_cam_rgb_.header.stamp = tf_world_enu_.header.stamp;
    tf_cam_rgb_.header.frame_id = camera_frame_id;
    tf_cam_rgb_.child_frame_id = camera_rgb_optical_frame_id;
    tf2::Quaternion q_cr;
    q_cr.setEuler(M_PI_2, 0, M_PI_2);
    tf_cam_rgb_.transform.rotation = tf2::toMsg(q_cr);

    // Put all static TFs in vector
    std::vector<geometry_msgs::msg::TransformStamped> static_tfs{tf_world_nwu_, tf_world_enu_, tf_cam_rgb_};

    // If requested, let's publish the TF and PoseWithCovarianceStamped for a static camera in the robot
    bool publish_camera_to_robot_tf_and_pose{false};
    this->declare_parameter("cameraStaticPublish.publish", publish_camera_to_robot_tf_and_pose);
    this->get_parameter("cameraStaticPublish.publish", publish_camera_to_robot_tf_and_pose);
    
    if (publish_camera_to_robot_tf_and_pose) {
        std::vector<double> publish_camera_tf_parameters;
        this->declare_parameter("cameraStaticPublish.TFParameters", publish_camera_tf_parameters);
        this->get_parameter("cameraStaticPublish.TFParameters", publish_camera_tf_parameters);

        std::string camera_pose_topic_name{"camera/pose"};
        this->declare_parameter("cameraStaticPublish.topic", camera_pose_topic_name);
        this->get_parameter("cameraStaticPublish.topic", camera_pose_topic_name);
        
        camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            camera_pose_topic_name, rclcpp::QoS(1).transient_local());

        std::string cam_rgb_pose_topic_name{"camera/pose_optical"};
        this->declare_parameter("cameraStaticPublish.pose_optical_topic", cam_rgb_pose_topic_name);
        this->get_parameter("cameraStaticPublish.pose_optical_topic", cam_rgb_pose_topic_name);
        
        cam_rgb_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            cam_rgb_pose_topic_name, rclcpp::QoS(1).transient_local());

        geometry_msgs::msg::TransformStamped tf_cam_rob_msg;
        tf_cam_rob_msg.header.frame_id = machine_frame_id;
        tf_cam_rob_msg.child_frame_id = camera_frame_id;
        tf_cam_rob_msg.header.stamp = tf_world_enu_.header.stamp;
        
        tf2::Transform tf_cam_rob;
        // First 3 parameters are for the translation, last 4 for rotation
        tf_cam_rob.setOrigin(tf2::Vector3(publish_camera_tf_parameters[0], publish_camera_tf_parameters[1],
                                         publish_camera_tf_parameters[2]));
        tf_cam_rob.setRotation(tf2::Quaternion(publish_camera_tf_parameters[3], publish_camera_tf_parameters[4],
                                              publish_camera_tf_parameters[5], publish_camera_tf_parameters[6]));

        // Transform to msg and add to vector
        tf_cam_rob_msg.transform = tf2::toMsg(tf_cam_rob);
        static_tfs.push_back(tf_cam_rob_msg);

        // Now the PoseWithCovariance equivalent
        cam_rob_pose_.header = tf_cam_rob_msg.header;
        cam_rob_pose_.pose.pose.position.x = tf_cam_rob.getOrigin().x();
        cam_rob_pose_.pose.pose.position.y = tf_cam_rob.getOrigin().y();
        cam_rob_pose_.pose.pose.position.z = tf_cam_rob.getOrigin().z();
        cam_rob_pose_.pose.pose.orientation.x = tf_cam_rob.getRotation().x();
        cam_rob_pose_.pose.pose.orientation.y = tf_cam_rob.getRotation().y();
        cam_rob_pose_.pose.pose.orientation.z = tf_cam_rob.getRotation().z();
        cam_rob_pose_.pose.pose.orientation.w = tf_cam_rob.getRotation().w();
        //TODO should we add uncertainty in the angles due to vibration?
        camera_pose_pub_->publish(cam_rob_pose_);

        // And the camRGB PoseWithCovariance equivalent
        rgb_cam_pose_.header = tf_cam_rgb_.header;
        rgb_cam_pose_.pose.pose.orientation = tf_cam_rgb_.transform.rotation;
        cam_rgb_pose_pub_->publish(rgb_cam_pose_);

        RCLCPP_INFO(this->get_logger(),
                   "Requested to publish camera->robot and optical->camera poses and transforms (unless disabled using parameter dontPublishTFs)");
    }

    // Broadcast all static tfs
    if (!dont_publish_tfs_)
        static_tf_broadcaster_->sendTransform(static_tfs);

    // Advertise std poses
    std_pose_.header.frame_id = world_frame_id;
    std_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(std_pose_topic_name, 10);

    // Advertise std raw poses
    std_raw_pose_.header.frame_id = world_frame_id;
    std_raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(std_raw_pose_topic_name, 10);

    // Advertise throttled poses
    throttled_pose_.header.frame_id = world_frame_id;
    throttled_pose_.header.stamp = this->now();
    throttled_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(throttled_pose_topic_name, 10);
    throttled_uav_pose_pub_ = this->create_publisher<uav_msgs::msg::UAVPose>(throttled_uav_pose_topic_name, 10);

    // Subscribe to poses
    pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        pose_topic_name, 10, std::bind(&TfFromUAVPose::poseCallback, this, std::placeholders::_1));
    raw_pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
        raw_pose_topic_name, 10, std::bind(&TfFromUAVPose::rawPoseCallback, this, std::placeholders::_1));
}

void TfFromUAVPose::poseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {

    // Copy contents to std pose msg
    std_pose_.header.stamp = msg->header.stamp;
    std_pose_.pose.pose.position = msg->position;
    std_pose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, std_pose_.pose);

    // Add offset
    try {
        std_pose_.pose.pose.position.x += offset_.at(0);
        std_pose_.pose.pose.position.y += offset_.at(1);
        std_pose_.pose.pose.position.z += offset_.at(2);

        std_pose_.pose.covariance[0] += added_covariance_.at(0);
        std_pose_.pose.covariance[7] += added_covariance_.at(1);
        std_pose_.pose.covariance[14] += added_covariance_.at(2);
    }
    catch (std::out_of_range &oor) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't add offset: " << oor.what());
        return;
    }

    // Publish std pose msg
    std_pose_pub_->publish(std_pose_);

        rclcpp::Time current_time(msg->header.stamp);
        rclcpp::Time last_time(throttled_pose_.header.stamp);
        rclcpp::Duration timediff = current_time - last_time; 
        if ((1.0/timediff.seconds()) <= throttle_rate_) {
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
    if (!dont_publish_tfs_)
        tf_broadcaster_->sendTransform(tf_pose_);
}

void TfFromUAVPose::rawPoseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg) {

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
    const std::vector<rclcpp::Parameter> &parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == "offsetX") {
            offset_[0] = param.as_double();
        } else if (param.get_name() == "offsetY") {
            offset_[1] = param.as_double();
        } else if (param.get_name() == "offsetZ") {
            offset_[2] = param.as_double();
        } else if (param.get_name() == "covarianceX") {
            added_covariance_[0] = param.as_double();
        } else if (param.get_name() == "covarianceY") {
            added_covariance_[1] = param.as_double();
        } else if (param.get_name() == "covarianceZ") {
            added_covariance_[2] = param.as_double();
        } else if (param.get_name() == "throttleRate") {
            throttle_rate_ = param.as_double();
        }
    }

    RCLCPP_INFO(this->get_logger(), "Received parameter update");
    RCLCPP_INFO_STREAM(this->get_logger(), "ThrottleRate: " << throttle_rate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Offset: [" << offset_[0] << ", " << offset_[1] << ", " << offset_[2] << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Extra covariance: [" << added_covariance_[0] << ", " << added_covariance_[1] << ", " << added_covariance_[2] << "]");

    return result;
}

void uavCovariance_to_rosCovariance(
  const uav_msgs::msg::UAVPose::SharedPtr &uav_msg,
  geometry_msgs::msg::PoseWithCovariance     &std_pose_cov)
{
    using namespace Eigen;

    // zero out the ROS covariance
    std::fill(std_pose_cov.covariance.begin(),
              std_pose_cov.covariance.end(), 0.0);

    // copy position variances
    std_pose_cov.covariance[0]  = uav_msg->covariance[0];
    std_pose_cov.covariance[7]  = uav_msg->covariance[11];
    std_pose_cov.covariance[14] = uav_msg->covariance[22];

    // build quaternion and its 4×4 covariance
    Quaterniond q(
      uav_msg->orientation.w,
      uav_msg->orientation.x,
      uav_msg->orientation.y,
      uav_msg->orientation.z
    );
    q.normalize();

    Matrix4d quat_cov;
    quat_cov << 
      uav_msg->covariance[66], uav_msg->covariance[67], uav_msg->covariance[68], uav_msg->covariance[69],
      uav_msg->covariance[76], uav_msg->covariance[77], uav_msg->covariance[78], uav_msg->covariance[79],
      uav_msg->covariance[86], uav_msg->covariance[87], uav_msg->covariance[88], uav_msg->covariance[89],
      uav_msg->covariance[96], uav_msg->covariance[97], uav_msg->covariance[98], uav_msg->covariance[99];

    // Jacobian for quaternion→Euler
    Matrix3d J = Matrix3d::Zero();
    const double qw=q.w(), qx=q.x(), qy=q.y(), qz=q.z(), norm2=qw*qw+qx*qx+qy*qy+qz*qz;
    J(0,0) = 2*(qy*qw+qx*qz)/norm2;
    J(0,1) = 2*(qz*qw-qx*qy)/norm2;
    J(0,2) = 2*(qx*qx+qw*qw-0.5)/norm2;
    J(1,0) = -2*qx/norm2;
    J(1,1) =  2*qw/norm2;
    J(2,0) =  2*(qx*qy+qz*qw)/norm2;
    J(2,1) =  2*(0.5 - qx*qx - qz*qz)/norm2;
    J(2,2) =  2*(qy*qz - qx*qw)/norm2;

    // Euler covariance
    Matrix3d euler_cov = J * quat_cov.block<3,3>(1,1) * J.transpose();

    std_pose_cov.covariance[21] = euler_cov(0,0);  // roll
    std_pose_cov.covariance[28] = euler_cov(1,1);  // pitch
    std_pose_cov.covariance[35] = euler_cov(2,2);  // yaw
}
}

int main(int argc, char **argv) {

    // Init node
    rclcpp::init(argc, argv);

    // Instance of class
    auto node = std::make_shared<tf_from_uav_pose::TfFromUAVPose>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}