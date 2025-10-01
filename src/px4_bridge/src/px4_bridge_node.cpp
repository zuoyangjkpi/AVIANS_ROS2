/**
 * @file px4_bridge_node.cpp
 * @brief Bridge node between AVIANS ROS2 system and PX4 autopilot
 *
 * This node subscribes to AVIANS topics and publishes to PX4 via uXRCE-DDS:
 * 1. Drone odometry (position, velocity, attitude)
 * 2. State machine status
 * 3. NMPC control commands
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// PX4 messages - these will be available when px4_msgs is installed
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class PX4BridgeNode : public rclcpp::Node
{
public:
    PX4BridgeNode() : Node("px4_bridge_node")
    {
        RCLCPP_INFO(this->get_logger(), "🚁 PX4 Bridge Node starting...");

        // Initialize subscribers from AVIANS system
        init_avians_subscribers();

        // Initialize publishers to PX4
        init_px4_publishers();

        // Timer for periodic publishing (100Hz for control loop)
        control_timer_ = this->create_wall_timer(
            10ms, std::bind(&PX4BridgeNode::control_loop, this));

        // Timer for status publishing (10Hz)
        status_timer_ = this->create_wall_timer(
            100ms, std::bind(&PX4BridgeNode::status_loop, this));

        RCLCPP_INFO(this->get_logger(), "✅ PX4 Bridge Node initialized");
        RCLCPP_INFO(this->get_logger(), "📡 Waiting for uXRCE-DDS connection to PX4...");
    }

private:
    void init_avians_subscribers()
    {
        // Subscribe to drone odometry from Gazebo/real sensors
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/X3/odometry", 10,
            std::bind(&PX4BridgeNode::odometry_callback, this, std::placeholders::_1));

        // Subscribe to NMPC control commands
        nmpc_control_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/drone/control/velocity_setpoint", 10,
            std::bind(&PX4BridgeNode::nmpc_control_callback, this, std::placeholders::_1));

        // Subscribe to waypoint commands
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/control/waypoint_command", 10,
            std::bind(&PX4BridgeNode::waypoint_callback, this, std::placeholders::_1));

        // Subscribe to state machine status
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drone/state", 10,
            std::bind(&PX4BridgeNode::state_callback, this, std::placeholders::_1));

        // Subscribe to NMPC status
        nmpc_status_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/drone/controller/status", 10,
            std::bind(&PX4BridgeNode::nmpc_status_callback, this, std::placeholders::_1));
    }

    void init_px4_publishers()
    {
        // PX4 publishers using px4_msgs
        // These topics are bridged via uXRCE-DDS to PX4 firmware

        // Offboard control mode - tells PX4 what we're controlling
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        // Trajectory setpoint - position/velocity/acceleration commands
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        // Vehicle command - for arming, mode changes, etc.
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Subscribe to PX4 status
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            std::bind(&PX4BridgeNode::px4_status_callback, this, std::placeholders::_1));
    }

    // ========== AVIANS Callbacks ==========

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
        odom_received_ = true;
    }

    void nmpc_control_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_control_ = msg;
        control_received_ = true;
    }

    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_waypoint_ = msg;
        waypoint_received_ = true;
    }

    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_state_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "State: %s", current_state_.c_str());
    }

    void nmpc_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // msg.data: [person_detected, tracking_distance, tracking_altitude, ...]
        if (msg->data.size() >= 3) {
            person_detected_ = (msg->data[0] > 0.5);
            tracking_distance_ = msg->data[1];
            tracking_altitude_ = msg->data[2];
        }
    }

    void px4_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        px4_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        px4_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    }

    // ========== Control Loop ==========

    void control_loop()
    {
        // Send offboard control mode (required for PX4 offboard mode)
        publish_offboard_control_mode();

        // Send control commands based on state
        if (current_state_ == "TRACK" && control_received_) {
            publish_velocity_setpoint();
        } else if (waypoint_received_) {
            publish_position_setpoint();
        }

        offboard_counter_++;
    }

    void status_loop()
    {
        if (odom_received_ && offboard_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "📊 State: %s | PX4: %s | Tracking: %.2fm",
                current_state_.c_str(),
                px4_offboard_ ? "OFFBOARD" : "MANUAL",
                tracking_distance_);
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Set control mode based on what we're sending
        if (current_state_ == "TRACK") {
            msg.position = false;
            msg.velocity = true;  // Velocity control for tracking
            msg.acceleration = false;
        } else {
            msg.position = true;   // Position control for waypoints
            msg.velocity = false;
            msg.acceleration = false;
        }
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_pub_->publish(msg);
    }

    void publish_velocity_setpoint()
    {
        if (!current_control_) return;

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Velocity setpoint from NMPC
        msg.velocity[0] = current_control_->twist.linear.x;  // Forward
        msg.velocity[1] = current_control_->twist.linear.y;  // Right
        msg.velocity[2] = current_control_->twist.linear.z;  // Down (NED frame)

        // Yaw rate
        msg.yawspeed = current_control_->twist.angular.z;

        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_position_setpoint()
    {
        if (!current_waypoint_) return;

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Position setpoint from waypoint controller
        // Convert from ENU (ROS) to NED (PX4)
        msg.position[0] = current_waypoint_->pose.position.x;   // North
        msg.position[1] = -current_waypoint_->pose.position.y;  // East (negated)
        msg.position[2] = -current_waypoint_->pose.position.z;  // Down (negated)

        trajectory_setpoint_pub_->publish(msg);
    }

    // ========== Member Variables ==========

    // Subscribers from AVIANS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nmpc_control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr nmpc_status_sub_;

    // Publishers to PX4
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscriber from PX4
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // State variables
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    geometry_msgs::msg::TwistStamped::SharedPtr current_control_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_waypoint_;
    std::string current_state_ = "IDLE";

    bool odom_received_ = false;
    bool control_received_ = false;
    bool waypoint_received_ = false;
    bool person_detected_ = false;
    bool px4_armed_ = false;
    bool px4_offboard_ = false;

    double tracking_distance_ = 0.0;
    double tracking_altitude_ = 0.0;
    uint64_t offboard_counter_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
