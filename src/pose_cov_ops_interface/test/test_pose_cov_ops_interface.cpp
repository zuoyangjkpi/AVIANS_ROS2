#include <rclcpp/rclcpp.hpp>
#include <pose_cov_ops_interface/pose_cov_ops_interface.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace pose_cov_ops::interface;
using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
public:
  TestNode() : Node("test_pose_cov_ops") {
    // Create publishers for test data
    robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/robot_pose", 10);
    
    // Initialize the interface with proper topic configuration
    init_interface();
    
    // Start publishing test data
    data_timer_ = this->create_wall_timer(
      100ms, [this]() { publish_test_data(); });
    
    // Start running tests after some data is available
    test_timer_ = this->create_wall_timer(
      2s, [this]() { run_tests(); });
  }

private:
  std::shared_ptr<Interface<std::string>> interface_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::TimerBase::SharedPtr test_timer_;
  int data_counter_ = 0;
  bool tests_run_ = false;

 void init_interface() {
    try {
        // Create topic configuration
        std::vector<topicSubInfo<std::string>> topics;
        
        // Initialize the topic info with all required parameters
        topics.emplace_back(
            "//machine_1/pose/raww/std",  // name
            "robot",        // key
            100,            // cache_size
            10,             // queue_size
            rclcpp::SystemDefaultsQoS()  // qos_profile
        );
        
        // Create interface with node shared pointer
        interface_ = std::make_shared<Interface<std::string>>(topics, shared_from_this());
        
        RCLCPP_INFO(get_logger(), "Interface initialized successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize interface: %s", e.what());
        rclcpp::shutdown();
    }
}

  void publish_test_data() {
    if (data_counter_ >= 50) {  // Stop after publishing 50 messages
      data_timer_->cancel();
      return;
    }

    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_msg->header.stamp = this->now();
    pose_msg->header.frame_id = "map";

    // Create a simple trajectory - robot moving in a circle
    double t = data_counter_ * 0.1;  // Time in seconds
    double radius = 2.0;
    
    pose_msg->pose.pose.position.x = radius * std::cos(t);
    pose_msg->pose.pose.position.y = radius * std::sin(t);
    pose_msg->pose.pose.position.z = 0.0;
    
    // Orientation facing the direction of movement
    double yaw = t + M_PI/2;  // 90 degrees ahead of position angle
    pose_msg->pose.pose.orientation.x = 0.0;
    pose_msg->pose.pose.orientation.y = 0.0;
    pose_msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose_msg->pose.pose.orientation.w = std::cos(yaw / 2.0);

    // Set realistic covariance values
    for (int i = 0; i < 36; ++i) {
      pose_msg->pose.covariance[i] = 0.0;
    }
    // Position covariance (x, y, z)
    pose_msg->pose.covariance[0] = 0.01;   // x variance
    pose_msg->pose.covariance[7] = 0.01;   // y variance
    pose_msg->pose.covariance[14] = 0.001; // z variance
    // Orientation covariance (roll, pitch, yaw)
    pose_msg->pose.covariance[21] = 0.001; // roll variance
    pose_msg->pose.covariance[28] = 0.001; // pitch variance
    pose_msg->pose.covariance[35] = 0.02;  // yaw variance

    robot_pose_pub_->publish(*pose_msg);
    
    if (data_counter_ % 10 == 0) {
      RCLCPP_INFO(get_logger(), "Published pose %d: x=%.2f, y=%.2f, yaw=%.2f", 
                  data_counter_, 
                  pose_msg->pose.pose.position.x,
                  pose_msg->pose.pose.position.y,
                  yaw);
    }
    
    data_counter_++;
  }

  void run_tests() {
    if (tests_run_ || !interface_) {
      return;
    }
    tests_run_ = true;
    
    RCLCPP_INFO(get_logger(), "\n=== Starting Pose Composition Tests ===");
    
    // Wait a bit more for data to be cached
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    test_basic_composition();
    test_interface_operations();
    test_covariance_propagation();
    test_print_interface();
    
    RCLCPP_INFO(get_logger(), "=== All tests completed ===");
  }

  void test_basic_composition() {
    RCLCPP_INFO(get_logger(), "\n--- Basic Composition Test ---");
    
    geometry_msgs::msg::PoseWithCovariance pose1, pose2, result;

    // Pose 1: Move 1 meter in X direction
    pose1.pose.position.x = 1.0;
    pose1.pose.position.y = 0.0;
    pose1.pose.position.z = 0.0;
    pose1.pose.orientation.w = 1.0;  // No rotation

    // Pose 2: Move 2 meters in Y direction  
    pose2.pose.position.x = 0.0;
    pose2.pose.position.y = 2.0;
    pose2.pose.position.z = 0.0;
    pose2.pose.orientation.w = 1.0;  // No rotation

    // Test composition
    pose_cov_ops::compose(pose1, pose2, result);
    
    RCLCPP_INFO(get_logger(), "Input pose1: (%.2f, %.2f, %.2f)", 
                pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
    RCLCPP_INFO(get_logger(), "Input pose2: (%.2f, %.2f, %.2f)", 
                pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z);
    RCLCPP_INFO(get_logger(), "Result:      (%.2f, %.2f, %.2f)", 
                result.pose.position.x, result.pose.position.y, result.pose.position.z);
    
    // Expected result should be (1, 2, 0)
    double expected_x = 1.0, expected_y = 2.0, expected_z = 0.0;
    double tolerance = 1e-6;
    
    if (std::abs(result.pose.position.x - expected_x) < tolerance &&
        std::abs(result.pose.position.y - expected_y) < tolerance &&
        std::abs(result.pose.position.z - expected_z) < tolerance) {
      RCLCPP_INFO(get_logger(), "✓ Basic composition test PASSED");
    } else {
      RCLCPP_ERROR(get_logger(), "✗ Basic composition test FAILED");
    }
  }

  void test_interface_operations() {
    RCLCPP_INFO(get_logger(), "\n--- Interface Operations Test ---");
    
    if (!interface_) {
      RCLCPP_ERROR(get_logger(), "✗ Interface not initialized");
      return;
    }

    try {
      // Test with current time
      auto current_time = this->now();
      geometry_msgs::msg::PoseWithCovariance input_pose, result;
      
      // Simple input pose
      input_pose.pose.position.x = 0.5;
      input_pose.pose.position.y = 0.5;
      input_pose.pose.position.z = 0.0;
      input_pose.pose.orientation.w = 1.0;

      // Test compose_up with single key
      bool success = interface_->compose_up(input_pose, "robot", current_time, result);
      
      if (success) {
        RCLCPP_INFO(get_logger(), "✓ compose_up succeeded");
        RCLCPP_INFO(get_logger(), "  Input:  (%.2f, %.2f, %.2f)", 
                    input_pose.pose.position.x, input_pose.pose.position.y, input_pose.pose.position.z);
        RCLCPP_INFO(get_logger(), "  Result: (%.2f, %.2f, %.2f)", 
                    result.pose.position.x, result.pose.position.y, result.pose.position.z);
      } else {
        RCLCPP_WARN(get_logger(), "✗ compose_up failed - no data available at current time");
      }

      // Test with a time in the past when we should have data
      auto past_time = current_time - rclcpp::Duration::from_seconds(1.0);
      success = interface_->compose_up(input_pose, "robot", past_time, result);
      
      if (success) {
        RCLCPP_INFO(get_logger(), "✓ compose_up with past time succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "✗ compose_up with past time failed - no data available");
      }

      // Test compose_down
      success = interface_->compose_down(input_pose, "robot", past_time, result);
      
      if (success) {
        RCLCPP_INFO(get_logger(), "✓ compose_down succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "✗ compose_down failed");
      }

      // Test with vector of keys (though we only have one)
      std::vector<std::string> keys = {"robot"};
      success = interface_->compose_up(input_pose, keys, past_time, result);
      
      if (success) {
        RCLCPP_INFO(get_logger(), "✓ compose_up with key vector succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "✗ compose_up with key vector failed");
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "✗ Interface operations test failed with exception: %s", e.what());
    }
  }

  void test_covariance_propagation() {
    RCLCPP_INFO(get_logger(), "\n--- Covariance Propagation Test ---");
    
    geometry_msgs::msg::PoseWithCovariance pose1, pose2, result;

    // Setup poses with known covariances
    pose1.pose.position.x = 1.0;
    pose1.pose.orientation.w = 1.0;
    pose2.pose.position.y = 1.0;
    pose2.pose.orientation.w = 1.0;

    // Initialize covariance matrices - set diagonal elements
    for (int i = 0; i < 36; ++i) {
      pose1.covariance[i] = 0.0;
      pose2.covariance[i] = 0.0;
    }
    
    // Set diagonal elements for position and orientation
    pose1.covariance[0] = 0.1;   // x variance
    pose1.covariance[7] = 0.1;   // y variance
    pose1.covariance[14] = 0.1;  // z variance
    pose1.covariance[35] = 0.1;  // yaw variance
    
    pose2.covariance[0] = 0.2;   // x variance
    pose2.covariance[7] = 0.2;   // y variance
    pose2.covariance[14] = 0.2;  // z variance
    pose2.covariance[35] = 0.2;  // yaw variance

    // Perform composition
    pose_cov_ops::compose(pose1, pose2, result);

    RCLCPP_INFO(get_logger(), "Input covariances:");
    RCLCPP_INFO(get_logger(), "  Pose1 diagonal: [%.3f, %.3f, %.3f, %.3f]", 
                pose1.covariance[0], pose1.covariance[7], pose1.covariance[14], pose1.covariance[35]);
    RCLCPP_INFO(get_logger(), "  Pose2 diagonal: [%.3f, %.3f, %.3f, %.3f]", 
                pose2.covariance[0], pose2.covariance[7], pose2.covariance[14], pose2.covariance[35]);
    RCLCPP_INFO(get_logger(), "Result covariance diagonal: [%.3f, %.3f, %.3f, %.3f]", 
                result.covariance[0], result.covariance[7], result.covariance[14], result.covariance[35]);

    // Check if result covariances are reasonable (should be >= input covariances)
    bool covariance_reasonable = true;
    if (result.covariance[0] < pose1.covariance[0] || result.covariance[0] < pose2.covariance[0]) {
      covariance_reasonable = false;
    }

    if (covariance_reasonable) {
      RCLCPP_INFO(get_logger(), "✓ Covariance propagation appears reasonable");
    } else {
      RCLCPP_WARN(get_logger(), "? Covariance propagation results may need verification");
    }
  }

  void test_print_interface() {
    RCLCPP_INFO(get_logger(), "\n--- Interface Status ---");
    
    if (interface_) {
      // Print interface status to console
      std::ostringstream oss;
      interface_->print(oss);
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      
      // Test oldestTimeInCache
      try {
        auto oldest_time = interface_->oldestTimeInCache("robot");
        RCLCPP_INFO(get_logger(), "Oldest time in cache: %.3f seconds", oldest_time.seconds());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to get oldest time: %s", e.what());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Interface not available");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<TestNode>();
    
    // Run for a limited time
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Run tests for 10 seconds
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      executor.spin_some(std::chrono::milliseconds(100));
      
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > std::chrono::seconds(10)) {
        RCLCPP_INFO(node->get_logger(), "Test completed, shutting down");
        break;
      }
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("test_node"), "Test failed with exception: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}