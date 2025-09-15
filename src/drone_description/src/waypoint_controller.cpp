#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <ros2_utils/clock_sync.hpp>

class PIDController3D {
public:
    PIDController3D() = default;
    
    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    
    void setLimits(double max_output, double max_integral) {
        max_output_ = max_output;
        max_integral_ = max_integral;
    }
    
    Eigen::Vector3d compute(const Eigen::Vector3d& error, double dt) {
        if (dt <= 0.0) return last_output_;
        
        // Proportional term
        Eigen::Vector3d p_term = kp_ * error;
        
        // Integral term with windup protection
        integral_error_ += error * dt;
        for (int i = 0; i < 3; ++i) {
            integral_error_[i] = std::clamp(integral_error_[i], -max_integral_, max_integral_);
        }
        Eigen::Vector3d i_term = ki_ * integral_error_;
        
        // Derivative term
        Eigen::Vector3d d_term = Eigen::Vector3d::Zero();
        if (last_error_initialized_) {
            d_term = kd_ * (error - last_error_) / dt;
        }
        
        // Total output
        Eigen::Vector3d output = p_term + i_term + d_term;
        
        // Apply output limits
        for (int i = 0; i < 3; ++i) {
            output[i] = std::clamp(output[i], -max_output_, max_output_);
        }
        
        // Store for next iteration
        last_error_ = error;
        last_error_initialized_ = true;
        last_output_ = output;
        
        return output;
    }
    
    void reset() {
        integral_error_ = Eigen::Vector3d::Zero();
        last_error_ = Eigen::Vector3d::Zero();
        last_error_initialized_ = false;
        last_output_ = Eigen::Vector3d::Zero();
    }

private:
    double kp_ = 1.0, ki_ = 0.0, kd_ = 0.0;
    double max_output_ = 1.0, max_integral_ = 1.0;
    Eigen::Vector3d integral_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_output_ = Eigen::Vector3d::Zero();
    bool last_error_initialized_ = false;
};

// Low-pass filter for command smoothing
class LowPassFilter {
public:
    LowPassFilter(double alpha = 0.8) : alpha_(alpha) {}
    
    Eigen::Vector3d filter(const Eigen::Vector3d& input) {
        if (!initialized_) {
            output_ = input;
            initialized_ = true;
        } else {
            output_ = alpha_ * output_ + (1.0 - alpha_) * input;
        }
        return output_;
    }
    
    void reset() {
        initialized_ = false;
        output_ = Eigen::Vector3d::Zero();
    }

private:
    double alpha_;
    Eigen::Vector3d output_ = Eigen::Vector3d::Zero();
    bool initialized_ = false;
};

class improved_waypoint_controller : public rclcpp::Node {
public:
    improved_waypoint_controller() : Node("improved_waypoint_controller") {
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", false);
        }
        
        // Log sim time status
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (use_sim_time) {
            RCLCPP_INFO(this->get_logger(), "Using simulation time");
        } else {
            RCLCPP_INFO(this->get_logger(), "Using system time");
        }
        
        // Declare parameters with improved defaults
        declare_parameter("max_horizontal_speed", 0.8);  // Reduced for stability
        declare_parameter("max_vertical_speed", 0.5);
        declare_parameter("max_yaw_rate", 0.3);          // Reduced for smoother yaw
        declare_parameter("waypoint_tolerance", 0.15);
        
        // Position PID gains
        declare_parameter("pos_kp", 1.2);               // Proportional gain
        declare_parameter("pos_ki", 0.1);               // Integral gain (small to prevent windup)
        declare_parameter("pos_kd", 0.3);               // Derivative gain for damping
        
        // Yaw PID gains  
        declare_parameter("yaw_kp", 0.8);               // Reduced P gain
        declare_parameter("yaw_ki", 0.05);              // Small integral term
        declare_parameter("yaw_kd", 0.15);              // Increased D gain for damping
        
        // Control parameters
        declare_parameter("prediction_time", 0.5);
        declare_parameter("enable_debug", true);
        declare_parameter("pure_tracking_mode", false);
        declare_parameter("command_filter_alpha", 0.7);  // Command smoothing
        declare_parameter("approach_distance", 1.0);     // Distance to start slowing down
        declare_parameter("min_speed_factor", 0.1);      // Minimum speed factor near waypoint
        
        // Initialize PID controllers
        setupPIDControllers();
        
        // Initialize filters
        command_filter_ = std::make_unique<LowPassFilter>(
            get_parameter("command_filter_alpha").as_double());
        
        // Enable controller
        enable_pub_ = create_publisher<std_msgs::msg::Bool>("/X3/enable", 10);
        
        // Velocity command publisher
        declare_parameter("cmd_vel_topic", "/X3/cmd_vel");
        std::string cmd_topic = get_parameter("cmd_vel_topic").as_string();
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

        // Subscribers
        declare_parameter("odom_topic", "/X3/odometry");
        std::string odom_topic = get_parameter("odom_topic").as_string();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Validate timestamp
                if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), 
                        rclcpp::Time(msg->header.stamp))) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Invalid timestamp in odometry");
                    return;
                }
                
                current_odom_ = msg;
                odom_received_ = true;
                if (target_received_) compute_improved_control();
            });

        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_waypoint", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                // Validate timestamp
                if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), 
                        rclcpp::Time(msg->header.stamp))) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Invalid timestamp in waypoint");
                    return;
                }
                
                updateTargetWaypoint(msg);
            });

        // Safety timer
        safety_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!target_received_) {
                    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
                }
            });
            
        // Initialize timing variables
        last_control_time_ = rclcpp::Time(0);
        previous_target_time_ = rclcpp::Time(0);
        
        RCLCPP_INFO(this->get_logger(), "Improved waypoint controller initialized");
    }

private:
    void setupPIDControllers() {
        // Position PID setup
        position_pid_.setGains(
            get_parameter("pos_kp").as_double(),
            get_parameter("pos_ki").as_double(),
            get_parameter("pos_kd").as_double()
        );
        position_pid_.setLimits(
            get_parameter("max_horizontal_speed").as_double(),
            1.0  // Max integral windup
        );
        
        // Yaw PID setup
        yaw_kp_ = get_parameter("yaw_kp").as_double();
        yaw_ki_ = get_parameter("yaw_ki").as_double();
        yaw_kd_ = get_parameter("yaw_kd").as_double();
    }
    
    void updateTargetWaypoint(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Calculate target velocity if we have a previous target
        if (target_received_) {
            auto dt = (now() - previous_target_time_).seconds();
            if (dt > 0.01) {
                target_velocity_ = Eigen::Vector3d(
                    (msg->pose.position.x - current_target_position_[0]) / dt,
                    (msg->pose.position.y - current_target_position_[1]) / dt,
                    (msg->pose.position.z - current_target_position_[2]) / dt);
            }
        } else {
            RCLCPP_INFO(get_logger(), "First target received - activating improved control");
            target_received_ = true;
            send_enable_command();
            
            // Reset PID controllers for clean start
            position_pid_.reset();
            yaw_integral_error_ = 0.0;
            last_yaw_error_ = 0.0;
            command_filter_->reset();
        }
        
        // Store previous target for velocity calculation
        previous_target_position_ = current_target_position_;
        previous_target_time_ = now();
        
        // Update current target
        current_target_position_ = {
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        };
        
        // Extract target yaw
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_target_yaw_ = yaw;
        
        if (odom_received_) {
            compute_improved_control();
        }
    }

    void send_enable_command() {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        enable_pub_->publish(msg);
    }

    void compute_improved_control() {
        if (!current_odom_ || !target_received_ || !odom_received_) return;

        auto current_time = this->get_clock()->now();
        double dt = 0.02;  // Default timestep
        
        // Calculate actual timestep
        if (last_control_time_.nanoseconds() > 0 && 
            ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), current_time)) {
            dt = (current_time - last_control_time_).seconds();
            if (dt > 0.1 || dt <= 0.0) {
                dt = 0.02;  // Fallback to default if timestep is invalid
            }
        }
        last_control_time_ = current_time;

        // Get current drone state
        Eigen::Vector3d current_position(
            current_odom_->pose.pose.position.x,
            current_odom_->pose.pose.position.y,
            current_odom_->pose.pose.position.z);
            
        Eigen::Vector3d current_velocity(
            current_odom_->twist.twist.linear.x,
            current_odom_->twist.twist.linear.y,
            current_odom_->twist.twist.linear.z);

        // Get current yaw
        tf2::Quaternion q(
            current_odom_->pose.pose.orientation.x,
            current_odom_->pose.pose.orientation.y,
            current_odom_->pose.pose.orientation.z,
            current_odom_->pose.pose.orientation.w);
        tf2::Matrix3x3 rot_matrix(q);
        double roll, pitch, current_yaw;
        rot_matrix.getRPY(roll, pitch, current_yaw);

        // Calculate target position with prediction
        Eigen::Vector3d target_position(
            current_target_position_[0],
            current_target_position_[1],
            current_target_position_[2]);
            
        if (get_parameter("pure_tracking_mode").as_bool() && target_velocity_.norm() > 0.1) {
            double prediction_time = get_parameter("prediction_time").as_double();
            target_position += target_velocity_ * prediction_time;
        }

        // Position control with improved PID
        Eigen::Vector3d position_error = target_position - current_position;
        double distance = position_error.norm();
        auto tolerance = get_parameter("waypoint_tolerance").as_double();
        
        geometry_msgs::msg::Twist cmd;

        // IMPROVED POSITION CONTROL
        if (distance > tolerance) {
            // Calculate desired velocity using PID
            Eigen::Vector3d desired_velocity_world = position_pid_.compute(position_error, dt);
            
            // Apply speed ramping based on distance to target
            double approach_distance = get_parameter("approach_distance").as_double();
            double min_speed_factor = get_parameter("min_speed_factor").as_double();
            double speed_factor = 1.0;
            
            if (distance < approach_distance) {
                speed_factor = std::max(min_speed_factor, distance / approach_distance);
                desired_velocity_world *= speed_factor;
            }
            
            // Transform to body frame
            Eigen::Matrix3d world_to_body;
            world_to_body = Eigen::AngleAxisd(-current_yaw, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d desired_velocity_body = world_to_body * desired_velocity_world;
            
            // Apply velocity limits
            double max_h = get_parameter("max_horizontal_speed").as_double();
            double max_v = get_parameter("max_vertical_speed").as_double();
            
            desired_velocity_body.x() = std::clamp(desired_velocity_body.x(), -max_h, max_h);
            desired_velocity_body.y() = std::clamp(desired_velocity_body.y(), -max_h, max_h);
            desired_velocity_body.z() = std::clamp(desired_velocity_body.z(), -max_v, max_v);
            
            // Apply command smoothing
            Eigen::Vector3d smoothed_velocity = command_filter_->filter(desired_velocity_body);
            
            cmd.linear.x = smoothed_velocity.x();
            cmd.linear.y = smoothed_velocity.y();
            cmd.linear.z = smoothed_velocity.z();
            
            if (get_parameter("enable_debug").as_bool()) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Distance: %.3f, Speed factor: %.3f, Cmd: [%.3f, %.3f, %.3f]", 
                    distance, speed_factor, cmd.linear.x, cmd.linear.y, cmd.linear.z);
            }
        } else {
            // Near waypoint - minimal movement with active damping
            cmd.linear.x = -current_velocity.x() * 0.1;  // Small damping
            cmd.linear.y = -current_velocity.y() * 0.1;
            cmd.linear.z = -current_velocity.z() * 0.1;
        }

        // IMPROVED YAW CONTROL with PID
        double desired_yaw = current_target_yaw_;
        
        // For tracking mode, point toward target velocity direction
        if (get_parameter("pure_tracking_mode").as_bool() && target_velocity_.norm() > 0.1) {
            desired_yaw = atan2(target_velocity_.y(), target_velocity_.x());
        } else if (distance > tolerance) {
            // Point toward target position when moving
            double prediction_time = get_parameter("prediction_time").as_double();
            Eigen::Vector3d predicted_position = target_position + target_velocity_ * prediction_time;
            desired_yaw = atan2(
                predicted_position.y() - current_position.y(),
                predicted_position.x() - current_position.x());
        }
        
        // Yaw error with proper wrapping
        double yaw_error = desired_yaw - current_yaw;
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        
        // Yaw PID controller
        double yaw_rate = (yaw_error - last_yaw_error_) / dt;
        yaw_integral_error_ += yaw_error * dt;
        yaw_integral_error_ = std::clamp(yaw_integral_error_, -1.0, 1.0);  // Anti-windup
        
        cmd.angular.z = std::clamp(
            yaw_kp_ * yaw_error + 
            yaw_ki_ * yaw_integral_error_ + 
            yaw_kd_ * yaw_rate,
            -get_parameter("max_yaw_rate").as_double(),
            get_parameter("max_yaw_rate").as_double());
        
        last_yaw_error_ = yaw_error;

        // Publish command with NaN protection
        if (std::isfinite(cmd.linear.x) && std::isfinite(cmd.linear.y) && 
            std::isfinite(cmd.linear.z) && std::isfinite(cmd.angular.z)) {
            cmd_vel_pub_->publish(cmd);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid command detected, sending zero velocity");
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        }
    }

    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr safety_timer_;

    // State variables
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::array<double, 3> current_target_position_;
    std::array<double, 3> previous_target_position_;
    double current_target_yaw_;
    bool target_received_ = false;
    bool odom_received_ = false;
    
    Eigen::Vector3d target_velocity_ = Eigen::Vector3d::Zero();
    rclcpp::Time previous_target_time_;
    rclcpp::Time last_control_time_;
    
    // PID controllers and filters
    PIDController3D position_pid_;
    std::unique_ptr<LowPassFilter> command_filter_;
    
    // Yaw PID variables
    double yaw_kp_, yaw_ki_, yaw_kd_;
    double yaw_integral_error_ = 0.0;
    double last_yaw_error_ = 0.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<improved_waypoint_controller>();
    
    WAIT_FOR_CLOCK_DELAYED(node);
    
    RCLCPP_INFO(node->get_logger(), "Improved waypoint controller started with synchronized clock");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}