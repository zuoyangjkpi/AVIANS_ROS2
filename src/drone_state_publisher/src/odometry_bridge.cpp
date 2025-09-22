#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <builtin_interfaces/msg/time.hpp>

class GazeboOdometryBridge : public rclcpp::Node {
public:
  GazeboOdometryBridge()
  : rclcpp::Node("gazebo_odometry_bridge"), have_prev_(false), prev_time_(0.0)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/X3/odometry_raw");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/X3/odometry");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "X3/base_link");

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_topic_, 10, std::bind(&GazeboOdometryBridge::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Relaying %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  static double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.child_frame_id = child_frame_;
    odom.pose = msg->pose;

    const double t = stamp_to_sec(msg->header.stamp);
    const auto & p = msg->pose.pose.position;
    if (have_prev_) {
      const double dt = t - prev_time_;
      if (dt > 1e-3) {
        odom.twist.twist.linear.x = (p.x - prev_pos_.x) / dt;
        odom.twist.twist.linear.y = (p.y - prev_pos_.y) / dt;
        odom.twist.twist.linear.z = (p.z - prev_pos_.z) / dt;
      }
    }

    pub_->publish(odom);

    prev_pos_ = p;
    prev_time_ = t;
    have_prev_ = true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string child_frame_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;

  geometry_msgs::msg::Point prev_pos_;
  bool have_prev_;
  double prev_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboOdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
