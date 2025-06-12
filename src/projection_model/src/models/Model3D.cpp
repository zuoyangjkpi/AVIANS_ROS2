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

} // namespace model_distance_from_height