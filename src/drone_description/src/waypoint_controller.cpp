#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

class waypoint_controller : public rclcpp::Node {
public:
  waypoint_controller() : Node("waypoint_controller") {
    // Parameters
    declare_parameter("max_horizontal_speed", 2.0);
    declare_parameter("max_vertical_speed", 1.0);
    declare_parameter("max_yaw_rate", 1.0);
    declare_parameter("waypoint_tolerance", 0.1);
    declare_parameter("yaw_p_gain", 0.5);
    declare_parameter("enable_debug", true);

    // Enable controller
    enable_pub_ = create_publisher<std_msgs::msg::Bool>("/X3/enable", 10);
    
    // Velocity command publisher
    declare_parameter("cmd_vel_topic", "/X3/cmd_vel");
    std::string cmd_topic = get_parameter("cmd_vel_topic").as_string();
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    RCLCPP_INFO(get_logger(), "Publishing velocity commands to: %s", cmd_topic.c_str());

    // Subscribers
    declare_parameter("odom_topic", "/X3/odom");
    std::string odom_topic = get_parameter("odom_topic").as_string();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
        odom_received_ = true;
        
        if (get_parameter("enable_debug").as_bool() && debug_counter_ % 50 == 0) {
          RCLCPP_INFO(get_logger(), "Odom: pos=[%.2f, %.2f, %.2f]", 
                     msg->pose.pose.position.x, 
                     msg->pose.pose.position.y, 
                     msg->pose.pose.position.z);
        }
        debug_counter_++;
        
        if (target_received_) {
          compute_body_frame_velocity();
        }
      });

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_waypoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!target_received_) {
          RCLCPP_INFO(get_logger(), "First target received - activating control");
          target_received_ = true;
          send_enable_command();
        }
        
        // Store position
        current_target_position_ = {
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z
        };
        
        // Convert quaternion to yaw
        tf2::Quaternion q(
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z,
          msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_target_yaw_ = yaw;
        
        RCLCPP_INFO(get_logger(), "New target: pos=[%.2f, %.2f, %.2f] yaw=%.2f rad (%.1f°)", 
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                   current_target_yaw_, current_target_yaw_ * 180.0/M_PI);
        
        compute_body_frame_velocity();
      });

    // Safety timer
    safety_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (!target_received_) {
          geometry_msgs::msg::Twist cmd;
          cmd_vel_pub_->publish(cmd);  // Zero twist by default
          
          if (safety_msg_counter_ % 50 == 0) {
            RCLCPP_WARN(get_logger(), "No target received - publishing safety stop commands");
          }
          safety_msg_counter_++;
        }
      });

    RCLCPP_INFO(get_logger(), "Waypoint controller initialized");
  }

private:
  void send_enable_command() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    enable_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Enable command sent to /X3/enable");
  }

  void compute_body_frame_velocity() {
  if (!current_odom_ || !target_received_ || !odom_received_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "Missing data - odom: %s, target: %s", 
                        odom_received_ ? "OK" : "MISSING",
                        target_received_ ? "OK" : "MISSING");
    return;
  }

  // Get current yaw from odometry
  tf2::Quaternion q(
    current_odom_->pose.pose.orientation.x,
    current_odom_->pose.pose.orientation.y,
    current_odom_->pose.pose.orientation.z,
    current_odom_->pose.pose.orientation.w);
  tf2::Matrix3x3 rot_matrix(q);
  double roll, pitch, current_yaw;
  rot_matrix.getRPY(roll, pitch, current_yaw);

  // Calculate position error
  Eigen::Vector3d world_error(
    current_target_position_[0] - current_odom_->pose.pose.position.x,
    current_target_position_[1] - current_odom_->pose.pose.position.y,
    current_target_position_[2] - current_odom_->pose.pose.position.z);
  
  double distance = world_error.norm();
  auto tolerance = get_parameter("waypoint_tolerance").as_double();
  
  geometry_msgs::msg::Twist cmd;

  if (distance > tolerance) {
    if (distance < 1e-6) {
      RCLCPP_WARN(get_logger(), "Distance to target too small (%.6f) - skipping velocity computation", distance);
      return;
    }

    // Convert error to body frame
    Eigen::Matrix3d world_to_body;
    world_to_body = Eigen::AngleAxisd(-current_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d body_error = world_to_body * world_error;

    if (body_error.norm() < 1e-6) {
      RCLCPP_WARN(get_logger(), "Body error too small (%.6f) - skipping velocity computation", body_error.norm());
      return;
    }

    Eigen::Vector3d body_direction = body_error.normalized();

    // Position control
    double speed_factor = std::min(1.0, distance / 2.0);
    double max_h = get_parameter("max_horizontal_speed").as_double();
    double max_v = get_parameter("max_vertical_speed").as_double();

    cmd.linear.x = std::clamp(body_direction.x() * max_h * speed_factor, -max_h, max_h);
    cmd.linear.y = std::clamp(body_direction.y() * max_h * speed_factor, -max_h, max_h);
    cmd.linear.z = std::clamp(body_direction.z() * max_v * speed_factor, -max_v, max_v);

    // Yaw control
    double desired_yaw = atan2(world_error.y(), world_error.x());
    double yaw_error = desired_yaw - current_yaw;

    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    double max_yaw = get_parameter("max_yaw_rate").as_double();
    double p_gain = get_parameter("yaw_p_gain").as_double();
    cmd.angular.z = std::clamp(yaw_error * p_gain, -max_yaw, max_yaw);
  } else {
    // Hold position, align yaw
    double yaw_error = current_target_yaw_ - current_yaw;
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    double max_yaw = get_parameter("max_yaw_rate").as_double();
    double p_gain = get_parameter("yaw_p_gain").as_double();
    cmd.angular.z = std::clamp(yaw_error * p_gain, -max_yaw, max_yaw);
  }

  // Final safety check
  if (std::isnan(cmd.linear.x) || std::isnan(cmd.linear.y) || std::isnan(cmd.linear.z) ||
      std::isnan(cmd.angular.z)) {
    RCLCPP_ERROR(get_logger(), "NaN detected in velocity command! Skipping publish.");
    return;
  }

  cmd_vel_pub_->publish(cmd);
}

  // Member variables
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  std::array<double, 3> current_target_position_;
  double current_target_yaw_;
  bool target_received_ = false;
  bool odom_received_ = false;
  
  // Debug counters
  int debug_counter_ = 0;
  int safety_msg_counter_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypoint_controller>());
  rclcpp::shutdown();
  return 0;
}
