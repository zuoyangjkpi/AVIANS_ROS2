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
        auto last_log_time = start_time;
        
        while (rclcpp::ok()) {
            // Check if we have valid simulation time (non-zero)
            rclcpp::Time current_time = node->get_clock()->now();
            if (current_time.nanoseconds() > 0) {
                RCLCPP_INFO(node->get_logger(), 
                           "✓ Valid simulation time received: %.3f seconds", 
                           current_time.seconds());
                           
                // ADDITIONAL: Wait a bit more to ensure stability
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return true;
            }
            
            // Check timeout
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > timeout_duration) {
                RCLCPP_ERROR(node->get_logger(), 
                            "⚠️ Timeout waiting for simulation clock after %.1f seconds. "
                            "Is Gazebo running? Is /clock topic being published?", 
                            timeout_sec);
                return false;
            }
            
            // Spin to process any pending callbacks (including /clock messages)
            rclcpp::spin_some(node);
            
            // Small sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // Progress indicator every 5 seconds - FIX: Convert to seconds properly
            if ((std::chrono::steady_clock::now() - last_log_time) > std::chrono::seconds(5)) {
                double elapsed_seconds = std::chrono::duration<double>(elapsed).count();
                RCLCPP_INFO(node->get_logger(), "Still waiting for simulation clock... (%.1fs)", elapsed_seconds);
                last_log_time = std::chrono::steady_clock::now();
            }
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
            auto current_time = node->get_clock()->now();
            if (current_time.nanoseconds() == 0) {
                RCLCPP_WARN(node->get_logger(), "Clock returned zero time");
            }
            return current_time;
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "Failed to get current time: %s", e.what());
            return rclcpp::Time(0);
        }
    }
    
    // NEW: Method to validate timestamps before critical operations
    static bool validateTimestamp(rclcpp::Node::SharedPtr node, const rclcpp::Time& timestamp) {
        if (timestamp.nanoseconds() == 0) {
            RCLCPP_WARN(node->get_logger(), "Invalid timestamp detected (zero time)");
            return false;
        }
        
        auto current_time = node->get_clock()->now();
        if (current_time.nanoseconds() == 0) {
            RCLCPP_WARN(node->get_logger(), "Current clock time is invalid");
            return false;
        }
        
        return true;
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