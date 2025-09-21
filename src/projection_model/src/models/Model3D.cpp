//
// Implementation of 3D distance-from-height model (ROS2 version)
//

#include <projection_model/models/Model3D.h>
#include <stdexcept>
#include <cmath>

namespace model_distance_from_height {

Model3D::Model3D(double hmean, double hvar) : height_model_(hmean, hvar) {
  if (hmean == 0) {
    throw std::invalid_argument("The height model mean can't be 0");
  }
}

double Model3D::compute_distance(const geometry_msgs::msg::Point& p, double ymin, double delta_y) {
  // z == 1/2*(p3*real_person_height*ymin - p2*real_person_height)/(ycenter - ymin)
  // z = H*(p3*ymin - p2)/(height_pixels)
  // where height_pixels = ymax-ymin

  if (delta_y == 0.0) {
    throw std::invalid_argument("delta_y cannot be zero in compute_distance");
  }
  return height_model_.mean * (p.z * ymin - p.y) / delta_y;
}

double Model3D::compute_dist_var(
    const geometry_msgs::msg::PoseWithCovariance& pose,
    double delta_y,
    double var_delta_y,
    double ymin,
    double var_ymin,
    double dist) 
{
  // TODO: update this simplified model
  if (delta_y == 0.0) {
    throw std::invalid_argument("delta_y cannot be zero in compute_dist_var");
  }
  return dist * dist * (height_model_.var / height_model_.mean + var_delta_y / delta_y);
}

double Model3D::estimate_person_height_from_detection(const neural_network_msgs::msg::NeuralNetworkDetection& detection) {
  // Estimate person height based on bounding box aspect ratio
  double box_width = std::abs(detection.xmax - detection.xmin);
  double box_height = std::abs(detection.ymax - detection.ymin);

  if (box_height == 0.0) {
    return height_model_.mean;  // Fallback to default height
  }

  double aspect_ratio = box_height / box_width;

  // Typical human aspect ratios in images:
  // Standing person: ~2.0-3.0 (height/width)
  // Crouching/sitting: ~1.0-1.5
  // Child: ~1.5-2.5

  double estimated_height;
  if (aspect_ratio > 2.5) {
    // Very tall/thin detection - likely adult standing
    estimated_height = 1.8;
  } else if (aspect_ratio > 2.0) {
    // Normal adult standing
    estimated_height = 1.7;
  } else if (aspect_ratio > 1.5) {
    // Shorter person or child
    estimated_height = 1.5;
  } else if (aspect_ratio > 1.0) {
    // Crouching or sitting person
    estimated_height = 1.3;
  } else {
    // Very wide detection - unusual, use default
    estimated_height = height_model_.mean;
  }

  // Weight the estimate with detection confidence
  double confidence_weight = std::min(1.0, std::max(0.5, static_cast<double>(detection.detection_score)));
  return confidence_weight * estimated_height + (1.0 - confidence_weight) * height_model_.mean;
}

double Model3D::compute_distance_adaptive(const geometry_msgs::msg::Point& p, double ymin, double delta_y, double adaptive_height) {
  // Same computation as compute_distance but using adaptive height instead of fixed height_model_.mean
  if (delta_y == 0.0) {
    throw std::invalid_argument("delta_y cannot be zero in compute_distance_adaptive");
  }
  return adaptive_height * (p.z * ymin - p.y) / delta_y;
}

} // namespace model_distance_from_height