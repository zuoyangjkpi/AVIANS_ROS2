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
  // Improved person height estimation with multiple factors
  double box_width = std::abs(detection.xmax - detection.xmin);
  double box_height = std::abs(detection.ymax - detection.ymin);

  if (box_height == 0.0 || box_width == 0.0) {
    return height_model_.mean;  // Fallback to default height
  }

  double aspect_ratio = box_height / box_width;
  double box_area = box_width * box_height;

  // Enhanced height estimation with multiple cues
  double estimated_height;

  // Primary estimation adapted for 30° downward camera tilt
  // With 30° tilt, people appear compressed vertically, so adjust thresholds
  if (aspect_ratio > 2.4) {  // Reduced from 2.8 due to tilt compression
    // Very tall/thin detection - adult standing with tilt compression
    estimated_height = 1.75 + 0.08 * std::min(2.0, (aspect_ratio - 2.4) / 0.4);
  } else if (aspect_ratio > 1.9) {  // Reduced from 2.2
    // Normal adult standing with 30° tilt compression
    double ratio_factor = (aspect_ratio - 1.9) / (2.4 - 1.9);
    estimated_height = 1.65 + ratio_factor * 0.1;  // Between 1.65-1.75m
  } else if (aspect_ratio > 1.6) {  // Reduced from 1.8
    // Shorter adult or teenager with tilt
    double ratio_factor = (aspect_ratio - 1.6) / (1.9 - 1.6);
    estimated_height = 1.5 + ratio_factor * 0.15;  // Between 1.5-1.65m
  } else if (aspect_ratio > 1.1) {  // Reduced from 1.2
    // Child or crouching person with tilt compression
    double ratio_factor = (aspect_ratio - 1.1) / (1.6 - 1.1);
    estimated_height = 1.2 + ratio_factor * 0.3;   // Between 1.2-1.5m
  } else {
    // Very wide detection - might be lying down or very compressed by tilt
    estimated_height = height_model_.mean * 0.7;  // More conservative for tilt
  }

  // Secondary correction based on detection size (larger boxes suggest closer/taller people)
  double size_correction = 1.0;
  if (box_area > 10000) {  // Large detection box
    size_correction = 1.05;  // Slightly increase height estimate
  } else if (box_area < 2000) {  // Small detection box
    size_correction = 0.95;  // Slightly decrease height estimate
  }

  estimated_height *= size_correction;

  // Apply confidence-based smoothing with better weighting
  double confidence = static_cast<double>(detection.detection_score);
  double confidence_weight = std::min(0.9, std::max(0.3, confidence));

  // Use temporal smoothing - bias towards default height for low confidence
  double final_height = confidence_weight * estimated_height +
                       (1.0 - confidence_weight) * height_model_.mean;

  // Clamp to reasonable human height range
  return std::max(0.8, std::min(2.2, final_height));
}

double Model3D::compute_distance_adaptive(const geometry_msgs::msg::Point& p, double ymin, double delta_y, double adaptive_height) {
  // Camera-specific computation with 30° downward tilt compensation
  if (delta_y == 0.0) {
    throw std::invalid_argument("delta_y cannot be zero in compute_distance_adaptive");
  }

  // Basic distance computation
  double basic_distance = adaptive_height * (p.z * ymin - p.y) / delta_y;

  // Camera tilt compensation for X3 drone (30° downward tilt = 0.5236 rad)
  const double CAMERA_TILT_ANGLE = 0.5236;  // 30 degrees down
  const double tilt_cos = cos(CAMERA_TILT_ANGLE);  // ~0.866
  const double tilt_sin = sin(CAMERA_TILT_ANGLE);  // ~0.5

  // Correct for camera tilt: when camera tilts down, apparent height decreases
  // Ground distance = camera_distance * cos(tilt) + height_offset * sin(tilt)
  double tilt_corrected_distance = basic_distance / tilt_cos;

  // Apply perspective correction based on corrected distance
  double perspective_correction = 1.0;
  if (tilt_corrected_distance > 2.0) {  // For X3's 30° tilt, adjust threshold
    // Reduced correction factor for tilted camera (3% per 10m instead of 5%)
    perspective_correction = 1.0 + 0.003 * (tilt_corrected_distance - 2.0);
  }

  // For tilted camera, people appear wider at distance, compensate accordingly
  double aspect_ratio_correction = 1.0;
  if (tilt_corrected_distance > 5.0) {
    // Compensate for aspect ratio changes due to perspective + tilt
    aspect_ratio_correction = 1.0 - 0.02 * (tilt_corrected_distance - 5.0);
    aspect_ratio_correction = std::max(0.85, aspect_ratio_correction);
  }

  // Apply all corrections
  double corrected_distance = tilt_corrected_distance * perspective_correction * aspect_ratio_correction;

  // Tighter bounds for X3 drone's tracking range (0.8-30m suitable for 30° tilt)
  corrected_distance = std::max(0.8, std::min(30.0, corrected_distance));

  return corrected_distance;
}

} // namespace model_distance_from_height