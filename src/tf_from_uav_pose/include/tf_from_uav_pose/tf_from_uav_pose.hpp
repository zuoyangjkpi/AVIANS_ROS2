//
// tf_from_uav_poses_node.hpp
// C++17 is required for ROS2. Deal with it!
//

#ifndef TF_FROM_UAV_POSE_HPP
#define TF_FROM_UAV_POSE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <uav_msgs/msg/uav_pose.hpp>

// Dynamic reconfigure equivalent for ROS2
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace tf_from_uav_pose {

    void uavCovariance_to_rosCovariance(const uav_msgs::msg::UAVPose::SharedPtr &uav_msg,
                                        geometry_msgs::msg::PoseWithCovariance &std_pose_cov);

    class TfFromUAVPose : public rclcpp::Node {

    private:
        // Subscribers and Publishers
        rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr pose_sub_;
        rclcpp::Subscription<uav_msgs::msg::UAVPose>::SharedPtr raw_pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr std_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr std_raw_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr throttled_pose_pub_;
        rclcpp::Publisher<uav_msgs::msg::UAVPose>::SharedPtr throttled_uav_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cam_rgb_pose_pub_;

        // Transform broadcasters
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

        // Message containers
        geometry_msgs::msg::PoseWithCovarianceStamped std_pose_, std_raw_pose_, cam_rob_pose_, rgb_cam_pose_;
        geometry_msgs::msg::PoseStamped throttled_pose_;
        geometry_msgs::msg::TransformStamped tf_pose_;
        geometry_msgs::msg::TransformStamped tf_world_enu_;
        geometry_msgs::msg::TransformStamped tf_world_nwu_;
        geometry_msgs::msg::TransformStamped tf_cam_rgb_;

        // Parameter callback handle
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        // Parameters
        std::vector<double> offset_{0, 0, 0};
        std::vector<double> added_covariance_{0, 0, 0};
        double throttle_rate_{10.0};
        bool dont_publish_tfs_{false};

    public:
        TfFromUAVPose();

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

        void poseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg);
        void rawPoseCallback(const uav_msgs::msg::UAVPose::SharedPtr msg);
    };
}

#endif //TF_FROM_UAV_POSE_HPP