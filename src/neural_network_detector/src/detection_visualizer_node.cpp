#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>

class DetectionVisualizerNode : public rclcpp::Node
{
public:
    DetectionVisualizerNode() : Node("detection_visualizer_node"),
        last_image_time_(this->get_clock()->now()),
        last_detection_time_(this->get_clock()->now()),
        last_publish_time_(this->get_clock()->now())
    {
        RCLCPP_INFO(this->get_logger(), "Detection visualizer node starting...");
        
        // Use a one-shot timer to initialize after the object is fully constructed
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DetectionVisualizerNode::initialize, this));
    }
    
    void initialize()
    {
        // Cancel the initialization timer
        init_timer_.reset();
        
        // Now it's safe to use shared_from_this()
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Subscribers
        image_sub_ = image_transport_->subscribe(
            "/camera/image_raw", 1, 
            std::bind(&DetectionVisualizerNode::imageCallback, this, std::placeholders::_1));
        
        detection_sub_ = this->create_subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
            "/person_detections", 1,
            std::bind(&DetectionVisualizerNode::detectionCallback, this, std::placeholders::_1));
        
        // Publisher for annotated image
        annotated_image_pub_ = image_transport_->advertise("/detection_image", 1);
        
        RCLCPP_INFO(this->get_logger(), "Detection visualizer node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /camera/image_raw and /person_detections");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /detection_image");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        current_image_ = msg;
        last_image_time_ = this->get_clock()->now();
        publishAnnotatedImage();
    }
    
    void detectionCallback(const neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr& msg)
    {
        current_detections_ = msg;
        last_detection_time_ = this->get_clock()->now();
        publishAnnotatedImage();
    }
    
    void publishAnnotatedImage()
    {
        if (!current_image_) {
            RCLCPP_DEBUG(this->get_logger(), "No current image available, skipping publish");
            return;
        }
        
        // Rate limiting to prevent excessive publishing
        auto now = this->get_clock()->now();
        if ((now - last_publish_time_).seconds() < 0.033) {  // Max 30 Hz for smoother display
            return;
        }
        last_publish_time_ = now;
        
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(current_image_, sensor_msgs::image_encodings::BGR8);
            
            // Draw bounding boxes if detections are available
            if (current_detections_ && !current_detections_->detections.empty()) {
                int detection_count = 0;
                for (const auto& detection : current_detections_->detections) {
                    if (detection.object_class == 0 && detection.detection_score > 0.3) { // Person class matching YOLO threshold
                        // Draw green bounding box with thicker line for stability
                        cv::rectangle(cv_ptr->image,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin)),
                            cv::Point(static_cast<int>(detection.xmax), static_cast<int>(detection.ymax)),
                            cv::Scalar(0, 255, 0), 4);
                        
                        // Draw detection score with fixed precision
                        char confidence_text[32];
                        snprintf(confidence_text, sizeof(confidence_text), "Person: %.2f", detection.detection_score);
                        cv::putText(cv_ptr->image, confidence_text,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin) - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                        detection_count++;
                    }
                }
                
                // Add detection count overlay
                if (detection_count > 0) {
                    std::string count_text = "Persons detected: " + std::to_string(detection_count);
                    cv::putText(cv_ptr->image, count_text,
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }
            } else {
                // Add "No detections" overlay when no person detected
                std::string no_detection_text = "No persons detected";
                cv::putText(cv_ptr->image, no_detection_text,
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
            
            // Always publish the image (with or without detections)
            annotated_image_pub_.publish(cv_ptr->toImageMsg());
            
            RCLCPP_DEBUG(this->get_logger(), "Published detection image");
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher annotated_image_pub_;
    
    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    sensor_msgs::msg::Image::ConstSharedPtr current_image_;
    neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr current_detections_;
    
    rclcpp::Time last_image_time_;
    rclcpp::Time last_detection_time_;
    rclcpp::Time last_publish_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}