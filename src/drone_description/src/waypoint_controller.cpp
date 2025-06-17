#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <ros2_utils/clock_sync.hpp>



class waypoint_controller : public rclcpp::Node {
public:
  waypoint_controller() : Node("waypoint_controller") {
    // Parameters
    declare_parameter("max_horizontal_speed", 1.0);
    declare_parameter("max_vertical_speed", 0.5);
    declare_parameter("max_yaw_rate", 0.5);
    declare_parameter("waypoint_tolerance", 0.15);
    declare_parameter("yaw_p_gain", 0.3);
    declare_parameter("yaw_d_gain", 0.1);
    declare_parameter("prediction_time", 0.5);
    declare_parameter("enable_debug", true);
    declare_parameter("pure_tracking_mode", false);

    // Enable controller
    enable_pub_ = create_publisher<std_msgs::msg::Bool>("/X3/enable", 10);
    
    // Velocity command publisher
    declare_parameter("cmd_vel_topic", "/X3/cmd_vel");
    std::string cmd_topic = get_parameter("cmd_vel_topic").as_string();
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    // Subscribers
    declare_parameter("odom_topic", "/X3/odom");
    std::string odom_topic = get_parameter("odom_topic").as_string();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
        odom_received_ = true;
        if (target_received_) compute_body_frame_velocity();
      });

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_waypoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (target_received_) {
          auto dt = (now() - previous_target_time_).seconds();
          if (dt > 0.01) {
            target_velocity_ = Eigen::Vector3d(
              (msg->pose.position.x - previous_target_position_[0]) / dt,
              (msg->pose.position.y - previous_target_position_[1]) / dt,
              (msg->pose.position.z - previous_target_position_[2]) / dt);
          }
        } else {
          RCLCPP_INFO(get_logger(), "First target received - activating control");
          target_received_ = true;
          send_enable_command();
        }
        
        previous_target_position_ = current_target_position_;
        previous_target_time_ = now();
        
        current_target_position_ = {
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z
        };
        
        tf2::Quaternion q(
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z,
          msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_target_yaw_ = yaw;
        
        compute_body_frame_velocity();
      });

    // Safety timer
    safety_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (!target_received_) {
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        }
      });
  }

private:
  void send_enable_command() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    enable_pub_->publish(msg);
  }

  void compute_body_frame_velocity() {
    if (!current_odom_ || !target_received_ || !odom_received_) return;

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

    // Calculate desired yaw
    double desired_yaw = current_target_yaw_;
    
    if (get_parameter("pure_tracking_mode").as_bool()) {
      if (target_velocity_.norm() > 0.1) {
        desired_yaw = atan2(target_velocity_.y(), target_velocity_.x());
      }
    } else if (distance > tolerance) {
      double prediction_time = get_parameter("prediction_time").as_double();
      Eigen::Vector3d predicted_position = {
        current_target_position_[0] + target_velocity_.x() * prediction_time,
        current_target_position_[1] + target_velocity_.y() * prediction_time,
        current_target_position_[2] + target_velocity_.z() * prediction_time
      };
      
      desired_yaw = atan2(
        predicted_position.y() - current_odom_->pose.pose.position.y,
        predicted_position.x() - current_odom_->pose.pose.position.x);
    }

    // Yaw control with PD
    double yaw_error = desired_yaw - current_yaw;
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    double yaw_rate = 0.0;
    if (last_yaw_error_time_.nanoseconds() > 0) {
      double dt = (now() - last_yaw_error_time_).seconds();
      if (dt > 0.01) {
        yaw_rate = (yaw_error - last_yaw_error_) / dt;
      }
    }
    
    cmd.angular.z = std::clamp(
      yaw_error * get_parameter("yaw_p_gain").as_double() - 
      yaw_rate * get_parameter("yaw_d_gain").as_double(), 
      -get_parameter("max_yaw_rate").as_double(), 
      get_parameter("max_yaw_rate").as_double());
    
    last_yaw_error_ = yaw_error;
    last_yaw_error_time_ = now();

    // Position control
    if (!get_parameter("pure_tracking_mode").as_bool() || distance > tolerance) {
      if (distance > tolerance) {
        Eigen::Matrix3d world_to_body ;
        world_to_body = Eigen::AngleAxisd(-current_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d body_error = world_to_body * world_error;
        Eigen::Vector3d body_direction = body_error.normalized();

        double speed_factor = std::min(1.0, distance / 2.0);
        double max_h = get_parameter("max_horizontal_speed").as_double();
        double max_v = get_parameter("max_vertical_speed").as_double();

        cmd.linear.x = std::clamp(body_direction.x() * max_h * speed_factor, -max_h, max_h);
        cmd.linear.y = std::clamp(body_direction.y() * max_h * speed_factor, -max_h, max_h);
        cmd.linear.z = std::clamp(body_direction.z() * max_v * speed_factor, -max_v, max_v);
      }
    }

    // Publish command
    if (!(std::isnan(cmd.linear.x)) && !(std::isnan(cmd.linear.y)) && 
        !(std::isnan(cmd.linear.z)) && !(std::isnan(cmd.angular.z))) {
      cmd_vel_pub_->publish(cmd);
    }
  }

  // Member variables
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  std::array<double, 3> current_target_position_;
  std::array<double, 3> previous_target_position_;
  double current_target_yaw_;
  bool target_received_ = false;
  bool odom_received_ = false;
  
  Eigen::Vector3d target_velocity_ = Eigen::Vector3d::Zero();
  rclcpp::Time previous_target_time_;
  
  double last_yaw_error_ = 0.0;
  rclcpp::Time last_yaw_error_time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<waypoint_controller>();
  WAIT_FOR_CLOCK_DELAYED(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}