#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace ros2_utils {

class ClockSynchronizer {
public:
    static bool waitForValidClock(rclcpp::Node::SharedPtr node, double timeout_sec = 30.0) {
        RCLCPP_INFO(node->get_logger(), "Waiting for valid clock...");
        
        // Check if using simulation time
        bool use_sim_time = false;
        try {
            use_sim_time = node->get_parameter("use_sim_time").as_bool();
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
            node->declare_parameter("use_sim_time", false);
            use_sim_time = node->get_parameter("use_sim_time").as_bool();
        }

        if (!use_sim_time) {
            RCLCPP_INFO(node->get_logger(), "Using system time - clock ready immediately");
            return true;
        }

        // Wait for simulation time to be available
        auto start_time = std::chrono::steady_clock::now();
        auto timeout_duration = std::chrono::duration<double>(timeout_sec);
        
        while (rclcpp::ok()) {
            // Check if we have valid simulation time (non-zero)
            rclcpp::Time current_time = node->get_clock()->now();
            if (current_time.nanoseconds() > 0) {
                RCLCPP_INFO(node->get_logger(), 
                           "Valid simulation time received: %.3f seconds", 
                           current_time.seconds());
                return true;
            }
            
            // Check timeout
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > timeout_duration) {
                RCLCPP_ERROR(node->get_logger(), 
                            "Timeout waiting for simulation clock after %.1f seconds. "
                            "Is Gazebo running? Is /clock topic being published?", 
                            timeout_sec);
                return false;
            }
            
            // Spin to process any pending callbacks (including /clock messages)
            rclcpp::spin_some(node);
            
            // Small sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }

    static bool initializeWithClockSync(rclcpp::Node::SharedPtr node, double timeout_sec = 30.0) {
        // Ensure use_sim_time parameter is properly declared
        if (!node->has_parameter("use_sim_time")) {
            node->declare_parameter("use_sim_time", false);
        }
        
        // Wait for valid clock
        if (!waitForValidClock(node, timeout_sec)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to synchronize with clock - node may not function correctly");
            return false;
        }
        
        RCLCPP_INFO(node->get_logger(), "Clock synchronization complete - node ready");
        return true;
    }

    static rclcpp::Time getSafeTime(rclcpp::Node::SharedPtr node) {
        try {
            return node->get_clock()->now();
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "Failed to get current time: %s", e.what());
            return rclcpp::Time(0);
        }
    }
};

// Convenience macros
#define WAIT_FOR_CLOCK_DELAYED(node_ptr) \
    do { \
        if (!ros2_utils::ClockSynchronizer::initializeWithClockSync(node_ptr)) { \
            throw std::runtime_error("Clock synchronization failed"); \
        } \
    } while(0)

} // namespace ros2_utils