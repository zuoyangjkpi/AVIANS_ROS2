#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>

class DetectionVisualizerNode : public rclcpp::Node
{
public:
    DetectionVisualizerNode() : Node("detection_visualizer_node")
    {
        // Create image transport
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
        
        RCLCPP_INFO(this->get_logger(), "Detection visualizer node started");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        current_image_ = msg;
        publishAnnotatedImage();
    }
    
    void detectionCallback(const neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr& msg)
    {
        current_detections_ = msg;
        publishAnnotatedImage();
    }
    
    void publishAnnotatedImage()
    {
        if (!current_image_) {
            return;
        }
        
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(current_image_, sensor_msgs::image_encodings::BGR8);
            
            // Draw bounding boxes if detections are available
            if (current_detections_) {
                for (const auto& detection : current_detections_->detections) {
                    if (detection.object_class == 1) { // Person class
                        // Draw green bounding box
                        cv::rectangle(cv_ptr->image,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin)),
                            cv::Point(static_cast<int>(detection.xmax), static_cast<int>(detection.ymax)),
                            cv::Scalar(0, 255, 0), 3);
                        
                        // Draw detection score
                        std::string confidence_text = "Person: " + std::to_string(detection.detection_score).substr(0, 4);
                        cv::putText(cv_ptr->image, confidence_text,
                            cv::Point(static_cast<int>(detection.xmin), static_cast<int>(detection.ymin) - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
            
            // Publish annotated image
            annotated_image_pub_.publish(cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher annotated_image_pub_;
    
    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_sub_;
    
    sensor_msgs::msg::Image::ConstSharedPtr current_image_;
    neural_network_msgs::msg::NeuralNetworkDetectionArray::ConstSharedPtr current_detections_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}