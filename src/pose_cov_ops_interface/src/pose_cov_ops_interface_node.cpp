#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseCovOpsInterfaceNode : public rclcpp::Node
{
public:
    PoseCovOpsInterfaceNode() : Node("pose_cov_ops_interface_node")
    {
        // Declare parameters
        this->declare_parameter("input_pose_topic", "/X3/odometry");
        this->declare_parameter("output_pose_topic", "/X3/pose_with_covariance");
        this->declare_parameter("default_covariance.matrix", std::vector<double>{
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        });

        // Get parameters
        input_topic_ = this->get_parameter("input_pose_topic").as_string();
        output_topic_ = this->get_parameter("output_pose_topic").as_string();
        auto cov_matrix = this->get_parameter("default_covariance.matrix").as_double_array();

        // Set up covariance matrix (6x6 = 36 elements)
        if (cov_matrix.size() != 36) {
            RCLCPP_WARN(this->get_logger(), "Covariance matrix should have 36 elements, using default");
            // Use default identity-like matrix
            for (int i = 0; i < 36; i++) {
                default_covariance_[i] = (i % 7 == 0) ? 0.1 : 0.0;  // Diagonal elements = 0.1
            }
        } else {
            for (int i = 0; i < 36; i++) {
                default_covariance_[i] = cov_matrix[i];
            }
        }

        // Create subscriber and publisher
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_, 10,
            std::bind(&PoseCovOpsInterfaceNode::odom_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "PoseCovOpsInterface node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert Odometry to PoseWithCovarianceStamped
        auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

        // Copy header
        pose_msg->header = msg->header;

        // Copy pose
        pose_msg->pose.pose = msg->pose.pose;

        // Set covariance - use odometry covariance if available, otherwise use default
        bool use_odom_cov = false;
        for (int i = 0; i < 36; i++) {
            if (msg->pose.covariance[i] != 0.0) {
                use_odom_cov = true;
                break;
            }
        }

        if (use_odom_cov) {
            for (int i = 0; i < 36; i++) {
                pose_msg->pose.covariance[i] = msg->pose.covariance[i];
            }
        } else {
            for (int i = 0; i < 36; i++) {
                pose_msg->pose.covariance[i] = default_covariance_[i];
            }
        }

        // Publish the message
        pose_pub_->publish(*pose_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    std::string input_topic_;
    std::string output_topic_;
    double default_covariance_[36];
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseCovOpsInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}