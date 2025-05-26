#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
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
    declare_parameter("waypoint_tolerance", 0.2);
    declare_parameter("enable_debug", true);

    // Enable controller
    enable_pub_ = create_publisher<std_msgs::msg::Bool>("/X3/enable", 10);
    
    // Velocity command publisher (body frame)
    // Try common topic names - adjust based on your Gazebo model configuration
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

    target_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/target_waypoint", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!target_received_) {
          RCLCPP_INFO(get_logger(), "First target received - activating control");
          target_received_ = true;
          // Send enable command when first target is received
          send_enable_command();
        }
        current_target_ = *msg;
        RCLCPP_INFO(get_logger(), "New target: [%.2f, %.2f, %.2f]", 
                   msg->x, msg->y, msg->z);
        compute_body_frame_velocity();
      });

    // Safety timer - but only send zero commands if we haven't received a target
    // This prevents interference with active control
    safety_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), // Reduced frequency to 10Hz
      [this]() {
        if (!target_received_) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.linear.z = 0.0;
          cmd.angular.x = 0.0;
          cmd.angular.y = 0.0;
          cmd.angular.z = 0.0;
          cmd_vel_pub_->publish(cmd);
          
          if (safety_msg_counter_ % 50 == 0) { // Print every 5 seconds
            RCLCPP_WARN(get_logger(), "No target received - publishing safety stop commands");
          }
          safety_msg_counter_++;
        }
      });

    RCLCPP_INFO(get_logger(), "Waypoint controller initialized");
    RCLCPP_INFO(get_logger(), "Listening for odometry on: %s", odom_topic.c_str());
    RCLCPP_INFO(get_logger(), "Waiting for target on /target_waypoint");
    
    // Diagnostic timer to check for missing odometry
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      [this, odom_topic]() {
        if (!odom_received_) {
          RCLCPP_ERROR(get_logger(), "No odometry received on %s - drone cannot move!", odom_topic.c_str());
          RCLCPP_ERROR(get_logger(), "Check if odometry topic exists and bridge is configured correctly");
        }
      });
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

    // Get parameters
    auto max_horz_speed = get_parameter("max_horizontal_speed").as_double();
    auto max_vert_speed = get_parameter("max_vertical_speed").as_double();
    auto max_yaw_rate = get_parameter("max_yaw_rate").as_double();
    auto tolerance = get_parameter("waypoint_tolerance").as_double();

    // Convert quaternion to rotation matrix
    tf2::Quaternion q(
      current_odom_->pose.pose.orientation.x,
      current_odom_->pose.pose.orientation.y,
      current_odom_->pose.pose.orientation.z,
      current_odom_->pose.pose.orientation.w);
    tf2::Matrix3x3 rot_matrix(q);
    
    // Get current yaw
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    
    // Calculate position error in world frame
    Eigen::Vector3d world_error(
      current_target_.x - current_odom_->pose.pose.position.x,
      current_target_.y - current_odom_->pose.pose.position.y,
      current_target_.z - current_odom_->pose.pose.position.z);

    double distance = world_error.norm();
    
    // Debug output
    if (get_parameter("enable_debug").as_bool() && control_debug_counter_ % 20 == 0) {
      RCLCPP_INFO(get_logger(), "Distance to target: %.3f m (tolerance: %.2f)", distance, tolerance);
      RCLCPP_INFO(get_logger(), "World error: [%.3f, %.3f, %.3f]", 
                 world_error.x(), world_error.y(), world_error.z());
      RCLCPP_INFO(get_logger(), "Current yaw: %.2f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);
    }
    control_debug_counter_++;

    geometry_msgs::msg::Twist cmd;

    if (distance > tolerance) {
      // Convert error to body frame
      Eigen::Matrix3d world_to_body;
      world_to_body = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ());
      Eigen::Vector3d body_error = world_to_body * world_error;

      // Normalized body-frame direction
      Eigen::Vector3d body_direction = body_error.normalized();

      // Calculate desired body-frame velocities with proportional control
      double speed_factor = std::min(1.0, distance / 2.0); // Reduce speed when close
      
      cmd.linear.x = std::clamp(body_direction.x() * max_horz_speed * speed_factor, 
                               -max_horz_speed, max_horz_speed);
      cmd.linear.y = std::clamp(body_direction.y() * max_horz_speed * speed_factor, 
                               -max_horz_speed, max_horz_speed);
      cmd.linear.z = std::clamp(body_direction.z() * max_vert_speed * speed_factor, 
                               -max_vert_speed, max_vert_speed);

      // Yaw control to face target
      double desired_world_yaw = atan2(world_error.y(), world_error.x());
      double yaw_error = desired_world_yaw - yaw;
      
      // Normalize yaw error to [-pi, pi]
      while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
      
      cmd.angular.z = std::clamp(yaw_error * 0.5, -max_yaw_rate, max_yaw_rate);

      // Debug velocity commands
      if (get_parameter("enable_debug").as_bool() && control_debug_counter_ % 20 == 0) {
        RCLCPP_INFO(get_logger(), "Body frame error: [%.3f, %.3f, %.3f]", 
                   body_error.x(), body_error.y(), body_error.z());
        RCLCPP_INFO(get_logger(), "Velocity cmd: linear=[%.3f, %.3f, %.3f], angular_z=%.3f", 
                   cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
      }
    } else {
      // Hold position - zero velocity commands
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.z = 0.0;
      
      if (get_parameter("enable_debug").as_bool() && control_debug_counter_ % 50 == 0) {
        RCLCPP_INFO(get_logger(), "At target waypoint - holding position");
      }
    }

    // Ensure all angular components are set
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;

    cmd_vel_pub_->publish(cmd);
  }

  // Member variables
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  geometry_msgs::msg::Point current_target_;
  bool target_received_ = false;
  bool odom_received_ = false;
  
  // Debug counters
  int debug_counter_ = 0;
  int control_debug_counter_ = 0;
  int safety_msg_counter_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypoint_controller>());
  rclcpp::shutdown();
  return 0;
}