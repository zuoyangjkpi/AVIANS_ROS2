//
// Model for computing distance from height in 3D (ROS2 version)
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_MODEL3D_H
#define MODEL_DISTANCE_FROM_HEIGHT_MODEL3D_H

#include "ModelBase.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <neural_network_msgs/msg/neural_network_detection.hpp>
#include <cassert>  // Using standard C++ assertions

namespace model_distance_from_height {

class Model3D : public ModelBase {
  /// Implements the 3d model for computing the distance to an object, given its height and center

public:
  struct height_model_uncertainty height_model_;

  Model3D() = default;
  explicit Model3D(double hmean, double hvar);

  double compute_distance(const geometry_msgs::msg::Point& p, double ymin, double delta_y);
  double estimate_person_height_from_detection(const neural_network_msgs::msg::NeuralNetworkDetection& detection);
  double compute_distance_adaptive(const geometry_msgs::msg::Point& p, double ymin, double delta_y, double adaptive_height);
  double compute_dist_var(const geometry_msgs::msg::PoseWithCovariance& pose,
                         double delta_y,
                         double var_delta_y,
                         double ymin,
                         double var_ymin,
                         double dist);
};

} // namespace model_distance_from_height

#endif // MODEL_DISTANCE_FROM_HEIGHT_MODEL3D_H