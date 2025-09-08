#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <neural_network_msgs/msg/neural_network_detection_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class DetectionVisualizerNode : public rclcpp::Node
{
public:
    DetectionVisualizerNode() : Node("detection_visualizer_node")
    {
        // Initialize image transport
        image_transport_ = std::make_unique<image_transport::ImageTransport>(
            rclcpp::Node::SharedPtr(this));
        
        // Subscribers
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&DetectionVisualizerNode::imageCallback, this, std::placeholders::_1));
        
        detection_sub_ = create_subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>(
            "/person_detections", 10,
            std::bind(&DetectionVisualizerNode::detectionCallback, this, std::placeholders::_1));
        
        // Publisher for visualization image
        vis_image_pub_ = image_transport_->advertise("/detection_image", 1);
        
        RCLCPP_INFO(this->get_logger(), "Detection Visualizer Node started");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_image_ = msg;
        publishVisualization();
    }

    void detectionCallback(const neural_network_msgs::msg::NeuralNetworkDetectionArray::SharedPtr msg)
    {
        latest_detections_ = msg;
        publishVisualization();
    }

    void publishVisualization()
    {
        if (!latest_image_ || !latest_detections_) {
            return;
        }

        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image.clone();

            // Draw detection boxes
            for (const auto& detection : latest_detections_->detections) {
                // Only visualize person detections (class 0 for person in COCO)
                if (detection.object_class == 0 || detection.object_class == 1) {
                    cv::Point2i top_left(detection.xmin, detection.ymin);
                    cv::Point2i bottom_right(detection.xmax, detection.ymax);
                    
                    // Draw green bounding box
                    cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 255, 0), 3);
                    
                    // Add confidence text
                    std::string label = "Person: " + std::to_string(int(detection.detection_score * 100)) + "%";
                    cv::Point2i text_pos(detection.xmin, detection.ymin - 10);
                    cv::putText(image, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                }
            }

            // Convert back to ROS message and publish
            sensor_msgs::msg::Image::SharedPtr vis_msg = cv_bridge::CvImage(
                latest_image_->header, "bgr8", image).toImageMsg();
            vis_image_pub_.publish(vis_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<neural_network_msgs::msg::NeuralNetworkDetectionArray>::SharedPtr detection_sub_;
    image_transport::Publisher vis_image_pub_;
    
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    neural_network_msgs::msg::NeuralNetworkDetectionArray::SharedPtr latest_detections_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}