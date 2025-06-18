//
// tf_from_uav_pose_ros2.hpp
// ROS2 migration of tf_from_uav_poses_node
// Migrated to maintain exact functionality with proper sim time handling
//

#ifndef TF_FROM_UAV_POSE_H
#define TF_FROM_UAV_POSE_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <uav_msgs/msg/uav_pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf_from_uav_pose {

    void uavCovariance_to_rosCovariance(const uav_msgs::msg::UAVPose::SharedPtr uav_msg,
                                        geometry_msgs::msg::PoseWithCovariance &std_pose_cov);

    class TfFromUAVPose : public rclcpp::Node {

    private:
        // Subscribers
        rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr pose_sub_;
        rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr raw_pose_sub_;
        
        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr std_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr std_raw_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr throttled_pose_pub_;
        rclcpp::Publisher<uav_msgs::msg::UAVPose>::SharedPtr throttled_uav_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cam_rgb_pose_pub_;
        

        // TF Broadcasters
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

        // Timer for camera pose publishing (for pose_cov_ops_interface cache reliability)
        rclcpp::TimerBase::SharedPtr camera_pose_timer_;

        // Message storage
        geometry_msgs::msg::PoseWithCovarianceStamped std_pose_, std_raw_pose_, cam_rob_pose_, rgb_cam_pose_;
        geometry_msgs::msg::PoseStamped throttled_pose_;
        geometry_msgs::msg::TransformStamped tf_pose_;
        geometry_msgs::msg::TransformStamped tf_world_enu_;
        geometry_msgs::msg::TransformStamped tf_world_nwu_;
        geometry_msgs::msg::TransformStamped tf_cam_rgb_;

        // Parameter callback handle
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        // Parameters (matching ROS1 exactly)
        std::vector<double> offset_{0.0, 0.0, 0.0};
        std::vector<double> added_covariance_{0.0, 0.0, 0.0};
        double throttle_rate_{10.0};
        bool dont_publish_tfs_{false};

        // Frame IDs (with ROS2 naming conventions)
        std::string machine_frame_id_{"machine_1"};
        std::string world_frame_id_{"world"};
        std::string world_enu_frame_id_{"world_ENU"};
        std::string world_nwu_frame_id_{"world_NWU"};
        std::string camera_frame_id_{"machine_1_camera_link"};
        std::string camera_optical_frame_id_{"machine_1_camera_rgb_optical_link"};

        // Topic names (configurable)
        std::string pose_topic_name_{"/machine_1/pose"};
        std::string raw_pose_topic_name_{"/machine_1/pose/raw"};
        std::string std_pose_topic_name_{"/machine_1/pose/corr/std"};
        std::string std_raw_pose_topic_name_{"/machine_1/pose/raww/std"};
        std::string throttled_pose_topic_name_{"/machine_1/throttledPose"};
        std::string throttled_uav_pose_topic_name_{"/machine_1/throttledUAVPose"};

        // Camera static publishing parameters
        bool publish_camera_to_robot_tf_and_pose_{true};
        std::vector<double> camera_tf_parameters_{0.18, 0.0, -0.07, 0.0, -0.38, 0.0, 0.924};
        geometry_msgs::msg::TransformStamped camera_transform_;

    public:
        TfFromUAVPose();

        // Method to update timestamps after clock sync
        void updateTimestampsAfterClockSync();

        // Parameter callback (replaces dynamic reconfigure)
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> & parameters);

        // Message callbacks
        void poseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg);
        void rawPoseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg);

    private:
        void initializeParameters();
        void initializePublishers();
        void initializeSubscribers();
        void setupStaticTransforms();
        void setupCameraTransforms();
    };
}

#endif // TF_FROM_UAV_POSE_H