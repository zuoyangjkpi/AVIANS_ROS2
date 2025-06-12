//
// Base model for distance from height calculations (ROS2 version)
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_MODELBASE_H
#define MODEL_DISTANCE_FROM_HEIGHT_MODELBASE_H

namespace model_distance_from_height {

struct height_model_simple {
  double mean;

  explicit height_model_simple(double _mean) : mean(_mean) {}
  height_model_simple() = default;
};

struct height_model_uncertainty {
  double mean;
  double var;

  height_model_uncertainty(double _mean, double _var) : mean(_mean), var(_var) {}
  height_model_uncertainty() = default;
};

class ModelBase {
  /// Base class for models that provide distance to an object in an image
  /// given information like: height model, camera angle, detected height in pixels,
  /// object center in pixels. Camera calibration should be handled outside the model.
  
  // Note: The original virtual methods are commented out in the original
  // Uncomment and implement in derived classes if needed:
  // virtual double compute_distance() = 0;
  // virtual double compute_uncertainty() = 0;
};

} // namespace model_distance_from_height

#endif // MODEL_DISTANCE_FROM_HEIGHT_MODELBASE_H