//
// Migrated to ROS2 from ROS1 version created by glawless on 10.05.17.
// Migration done by Mohamed Abdelmawgoud on 12.06.25 .
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_PROJECTOR_H
#define MODEL_DISTANCE_FROM_HEIGHT_PROJECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>
#include <neural_network_msgs/msg/neural_network_feedback.hpp>

#include <image_geometry/pinhole_camera_model.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <projection_model/models/Model3D.h>
#include <pose_cov_ops_interface/pose_cov_ops_interface.h>

#include <memory>
#include <vector>
#include <string>

// For dynamic reconfigure in ROS2, we'll need to implement parameter callbacks
// This would typically use rcl_interfaces/msg/parameter_event.hpp

namespace model_distance_from_height {

enum class Poses : int {
  robot = 0,
  gpsoffset = 1,
  camera = 2,
  optical = 3
};

class Projector : public rclcpp::Node  {
public:
      Projector();
      virtual ~Projector() = default;

      void init();  // Add a separate init method

private:
  // ROS2 subscribers
  rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tracker_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr object_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_debug_pub_;
  rclcpp::Publisher<neural_network_msgs::msg::NeuralNetworkFeedback>::SharedPtr feedback_pub_;

#ifdef DEBUG_PUBLISHERS
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr debug_pub_;
#endif

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters);
  rclcpp::TimerBase::SharedPtr camera_info_timeout_;


  // Camera model
  image_geometry::PinholeCameraModel cameraModel_;

  // Projection model
  std::unique_ptr<Model3D> projectionModel_;

  // Interface for pose composition
  std::vector<pose_cov_ops::interface::topicSubInfo<int>> topics_;
  std::unique_ptr<pose_cov_ops::interface::Interface<int>> interface_;

  // Parameters
  double head_uncertainty_scale_;
  double feet_uncertainty_scale_;




  // Callbacks
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectionCallback3D(const neural_network_msgs::msg::NeuralNetworkDetectionArray::SharedPtr msg);
  void trackerCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Utility functions
  void interface_warning() const;
  bool detectBackwardsTimeJump();

  // Time tracking for backwards jump detection
  rclcpp::Time last_time_;
};

} // namespace model_distance_from_height

#endif // MODEL_DISTANCE_FROM_HEIGHT_PROJECTOR_H