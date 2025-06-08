#include "neural_network_detector/yolo12_detector_node.hpp"

namespace yolo12_detector_node
{

YOLO12DetectorNode::YOLO12DetectorNode(const rclcpp::NodeOptions & options)
: Node("yolo12_detector_node", options),
  feedback_received_(false),
  last_feedback_time_(this->now()),
  last_detection_time_(this->now())
{
    // Initialize parameters
    initializeParameters();
    
    // Initialize the YOLO12 detector
    initializeDetector();
    
    // Create subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 
        rclcpp::QoS(1).best_effort(),
        std::bind(&YOLO12DetectorNode::imageCallback, this, std::placeholders::_1));
    
    feedback_sub_ = this->create_subscription<neural_network_detector::msg::NeuralNetworkFeedback>(
        "feedback",
        rclcpp::QoS(10),
        std::bind(&YOLO12DetectorNode::feedbackCallback, this, std::placeholders::_1));
    
    // Create publishers
    detection_pub_ = this->create_publisher<neural_network_detector::msg::NeuralNetworkDetectionArray>(
        "detections", rclcpp::QoS(10));
    
    detection_count_pub_ = this->create_publisher<neural_network_detector::msg::NeuralNetworkNumberOfDetections>(
        "detection_count", rclcpp::QoS(10));
    
    if (publish_debug_image_) {
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "debug_image", rclcpp::QoS(1));
    }
    
    // Set up rate limiting
    if (max_update_rate_hz_ > 0.0) {
        min_detection_interval_ = rclcpp::Duration::from_seconds(1.0 / max_update_rate_hz_);
    } else {
        min_detection_interval_ = rclcpp::Duration::from_seconds(0.0);
    }
    
    RCLCPP_INFO(this->get_logger(), "YOLO12 Detector Node initialized successfully");
}

void YOLO12DetectorNode::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter<std::string>("model_path", "../third_party/YOLOs-CPP/models/yolo12n.onnx");
    this->declare_parameter<std::string>("labels_path", "../third_party/YOLOs-CPP/models/coco.names");
    this->declare_parameter<bool>("use_gpu", false);
    this->declare_parameter<float>("confidence_threshold", 0.5f);
    this->declare_parameter<float>("iou_threshold", 0.45f);
    this->declare_parameter<int>("desired_class", 1);
    this->declare_parameter<int>("desired_width", 300);
    this->declare_parameter<int>("desired_height", 300);
    this->declare_parameter<float>("aspect_ratio", 1.33333f);
    this->declare_parameter<double>("border_dropoff", 0.05);
    this->declare_parameter<bool>("publish_debug_image", false);
    this->declare_parameter<bool>("max_update_force", false);
    this->declare_parameter<double>("max_update_rate_hz", 0.0);
    this->declare_parameter<double>("feedback_timeout_sec", 5.0);
    
    // Variance parameters
    this->declare_parameter<float>("var_const_x_min", 0.0f);
    this->declare_parameter<float>("var_const_x_max", 0.0f);
    this->declare_parameter<float>("var_const_y_min", 0.0f);
    this->declare_parameter<float>("var_const_y_max", 0.0f);
    
    // Get parameter values
    model_path_ = this->get_parameter("model_path").as_string();
    labels_path_ = this->get_parameter("labels_path").as_string();
    use_gpu_ = this->get_parameter("use_gpu").as_bool();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    iou_threshold_ = this->get_parameter("iou_threshold").as_double();
    desired_class_ = this->get_parameter("desired_class").as_int();
    
    int width = this->get_parameter("desired_width").as_int();
    int height = this->get_parameter("desired_height").as_int();
    desired_resolution_ = cv::Size(width, height);
    
    aspect_ratio_ = this->get_parameter("aspect_ratio").as_double();
    border_dropoff_ = this->get_parameter("border_dropoff").as_double();
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    max_update_force_ = this->get_parameter("max_update_force").as_bool();
    max_update_rate_hz_ = this->get_parameter("max_update_rate_hz").as_double();
    
    double timeout_sec = this->get_parameter("feedback_timeout_sec").as_double();
    feedback_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);
    
    var_const_x_min_ = this->get_parameter("var_const_x_min").as_double();
    var_const_x_max_ = this->get_parameter("var_const_x_max").as_double();
    var_const_y_min_ = this->get_parameter("var_const_y_min").as_double();
    var_const_y_max_ = this->get_parameter("var_const_y_max").as_double();
    
    // Validate required parameters
    if (model_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "model_path parameter is required");
        throw std::runtime_error("model_path parameter is required");
    }
    
    if (labels_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "labels_path parameter is required");
        throw std::runtime_error("labels_path parameter is required");
    }
    
    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Model path: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Labels path: %s", labels_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Use GPU: %s", use_gpu_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Confidence threshold: %.2f", confidence_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Desired class: %d", desired_class_);
}

void YOLO12DetectorNode::initializeDetector()
{
    try {
        yolo_detector_ = std::make_unique<YOLO12Detector>(model_path_, labels_path_, use_gpu_);
        RCLCPP_INFO(this->get_logger(), "YOLO12 detector initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO12 detector: %s", e.what());
        throw;
    }
}

void YOLO12DetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Check rate limiting
    if (shouldSkipDetection()) {
        return;
    }
    
    try {
        // Convert ROS image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        if (image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }
        
        cv::Mat processing_image = image.clone();
        cv::Point crop_offset(0, 0);
        
        // Apply cropping if feedback is available and valid
        if (feedback_received_ && !isFeedbackTimedOut()) {
            cv::Rect crop_area = getCropArea(image.size(), latest_feedback_);
            if (crop_area.width > 0 && crop_area.height > 0 && 
                crop_area.x >= 0 && crop_area.y >= 0 && 
                crop_area.x + crop_area.width <= image.cols && 
                crop_area.y + crop_area.height <= image.rows) {
                
                processing_image = image(crop_area);
                crop_offset = cv::Point(crop_area.x, crop_area.y);
                
                RCLCPP_DEBUG(this->get_logger(), "Applied crop: (%d,%d) %dx%d", 
                           crop_area.x, crop_area.y, crop_area.width, crop_area.height);
            }
        }
        
        // Run YOLO12 detection
        std::vector<Detection> detections = yolo_detector_->detect(
            processing_image, confidence_threshold_, iou_threshold_);
        
        // Filter detections
        std::vector<Detection> filtered_detections = filterDetections(detections);
        
        // Convert to ROS message
        auto detection_msg = convertDetectionsToROS(filtered_detections, msg->header, crop_offset);
        
        // Publish detection results
        detection_pub_->publish(detection_msg);
        
        // Publish detection count
        auto count_msg = neural_network_detector::msg::NeuralNetworkNumberOfDetections();
        count_msg.header = msg->header;
        count_msg.data = static_cast<uint16_t>(filtered_detections.size());
        detection_count_pub_->publish(count_msg);
        
        // Publish debug image if enabled
        if (publish_debug_image_ && debug_image_pub_) {
            cv::Mat debug_image = processing_image.clone();
            yolo_detector_->drawBoundingBox(debug_image, filtered_detections);
            
            sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(
                msg->header, "bgr8", debug_image).toImageMsg();
            debug_image_pub_->publish(*debug_msg);
        }
        
        // Update timing
        last_detection_time_ = this->get_clock()->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Processed image with %zu detections", 
                    filtered_detections.size());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
}

void YOLO12DetectorNode::feedbackCallback(
    const neural_network_detector::msg::NeuralNetworkFeedback::SharedPtr msg)
{
    latest_feedback_ = *msg;
    feedback_received_ = true;
    last_feedback_time_ = this->get_clock()->now();
    
    RCLCPP_DEBUG(this->get_logger(), "Received feedback: ymin=%d, ymax=%d, xcenter=%d", 
                msg->ymin, msg->ymax, msg->xcenter);
}

cv::Rect YOLO12DetectorNode::getCropArea(
    const cv::Size& original_resolution, 
    const neural_network_detector::msg::NeuralNetworkFeedback& feedback)
{
    // Calculate crop area based on feedback information
    // This is a simplified version - you may need to adapt based on your specific requirements
    
    int center_x = feedback.xcenter;
    int y_min = feedback.ymin;
    int y_max = feedback.ymax;
    
    // Calculate crop dimensions
    int crop_height = std::max(1, y_max - y_min);
    int crop_width = static_cast<int>(crop_height * aspect_ratio_);
    
    // Calculate crop position
    int crop_x = std::max(0, center_x - crop_width / 2);
    int crop_y = std::max(0, y_min);
    
    // Ensure crop area is within image bounds
    crop_x = std::min(crop_x, original_resolution.width - crop_width);
    crop_y = std::min(crop_y, original_resolution.height - crop_height);
    crop_width = std::min(crop_width, original_resolution.width - crop_x);
    crop_height = std::min(crop_height, original_resolution.height - crop_y);
    
    // Apply border dropoff
    int border_x = static_cast<int>(crop_width * border_dropoff_);
    int border_y = static_cast<int>(crop_height * border_dropoff_);
    
    crop_x = std::max(0, crop_x - border_x);
    crop_y = std::max(0, crop_y - border_y);
    crop_width = std::min(original_resolution.width - crop_x, crop_width + 2 * border_x);
    crop_height = std::min(original_resolution.height - crop_y, crop_height + 2 * border_y);
    
    return cv::Rect(crop_x, crop_y, crop_width, crop_height);
}

neural_network_detector::msg::NeuralNetworkDetectionArray YOLO12DetectorNode::convertDetectionsToROS(
    const std::vector<Detection>& detections,
    const std_msgs::msg::Header& header,
    const cv::Point& crop_offset)
{
    auto detection_array = neural_network_detector::msg::NeuralNetworkDetectionArray();
    detection_array.header = header;
    
    for (const auto& detection : detections) {
        auto ros_detection = neural_network_detector::msg::NeuralNetworkDetection();
        ros_detection.header = header;
        
        // Adjust coordinates for crop offset
        ros_detection.xmin = static_cast<int16_t>(detection.box.x + crop_offset.x);
        ros_detection.xmax = static_cast<int16_t>(detection.box.x + detection.box.width + crop_offset.x);
        ros_detection.ymin = static_cast<int16_t>(detection.box.y + crop_offset.y);
        ros_detection.ymax = static_cast<int16_t>(detection.box.y + detection.box.height + crop_offset.y);
        
        ros_detection.object_class = static_cast<int16_t>(detection.classId);
        ros_detection.detection_score = detection.conf;
        
        // Set variance values
        ros_detection.variance_xmin = var_const_x_min_;
        ros_detection.variance_xmax = var_const_x_max_;
        ros_detection.variance_ymin = var_const_y_min_;
        ros_detection.variance_ymax = var_const_y_max_;
        
        detection_array.detections.push_back(ros_detection);
    }
    
    return detection_array;
}

bool YOLO12DetectorNode::isFeedbackTimedOut() const
{
    if (!feedback_received_) {
        return true;
    }
    
    auto current_time = this->get_clock()->now();
    auto time_since_feedback = current_time - last_feedback_time_;
    return time_since_feedback > feedback_timeout_;
}

bool YOLO12DetectorNode::shouldSkipDetection() const
{
    if (!max_update_force_ || min_detection_interval_.nanoseconds() == 0) {
        return false;
    }
    
    auto current_time = this->get_clock()->now();
    auto time_since_last = current_time - last_detection_time_;
    return time_since_last < min_detection_interval_;
}

std::vector<Detection> YOLO12DetectorNode::filterDetections(
    const std::vector<Detection>& detections) const
{
    std::vector<Detection> filtered;
    
    for (const auto& detection : detections) {
        // Filter by class if desired_class is specified (>= 0)
        if (desired_class_ >= 0 && detection.classId != desired_class_) {
            continue;
        }
        
        // Filter by confidence threshold
        if (detection.conf < confidence_threshold_) {
            continue;
        }
        
        filtered.push_back(detection);
    }
    
    return filtered;
}

} // namespace yolo12_detector_node


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        // Pass NodeOptions explicitly
        auto node = std::make_shared<yolo12_detector_node::YOLO12DetectorNode>(
            rclcpp::NodeOptions()
        );
        
        RCLCPP_INFO(node->get_logger(), "Starting YOLO12 Detector Node...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo12_detector_node"), 
                     "Failed to start node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
