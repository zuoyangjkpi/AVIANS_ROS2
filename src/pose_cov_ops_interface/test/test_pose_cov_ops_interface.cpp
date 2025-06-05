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
    // Delay interface construction using a timer to allow shared_from_this()
    init_timer_ = this->create_wall_timer(
      100ms, [this]() {
        if (!interface_) {
          // Properly initialize topics vector with all required arguments
          std::vector<topicSubInfo<std::string>> topics = {
            {"tf_pose", "robot", 10, 100, rclcpp::QoS(100)}
          };
          interface_ = std::make_shared<Interface<std::string>>(topics, shared_from_this());

          populate_test_data();
          test_basic_composition();
          test_covariance_propagation();
          test_timed_poses();
          test_identity_operations();
        }
      });
  }

private:
  std::shared_ptr<Interface<std::string>> interface_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  void populate_test_data() {
    auto now = this->now();

    for (int i = 0; i < 20; ++i) {
      auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pose->header.stamp = now + rclcpp::Duration::from_seconds(i * 0.5);
      pose->header.frame_id = "map";

      pose->pose.pose.position.x = 0.1 * i * 0.5;

      for (int j = 0; j < 36; j += 6) {
        pose->pose.covariance[j] = 0.01 * (i + 1);
      }

      RCLCPP_INFO(get_logger(), "Adding pose at time=%.2f, x=%.3f",
                  rclcpp::Time(pose->header.stamp).seconds(),
                  pose->pose.pose.position.x);

      interface_->addPose("robot", pose);
    }
  }

  void test_basic_composition() {
    geometry_msgs::msg::PoseWithCovariance pose1, pose2, result;

    pose1.pose.position.x = 1.0;
    pose1.pose.orientation.w = 1.0;
    pose2.pose.position.y = 2.0;
    pose2.pose.orientation.w = 1.0;

    pose_cov_ops::compose(pose1, pose2, result);
    RCLCPP_INFO(get_logger(), "\nBasic Composition Test:");
    RCLCPP_INFO(get_logger(), "  Input: (1,0,0) + (0,2,0)");
    RCLCPP_INFO(get_logger(), "  Result: (%.2f,%.2f,%.2f)",
                result.pose.position.x,
                result.pose.position.y,
                result.pose.position.z);
  }

  void test_covariance_propagation() {
    geometry_msgs::msg::PoseWithCovariance pose1, pose2, result;

    pose1.pose.position.x = 1.0;
    pose1.pose.orientation.w = 1.0;
    pose2.pose.position.y = 2.0;
    pose2.pose.orientation.w = 1.0;

    // Initialize diagonal covariance entries
    for (int i = 0; i < 36; i += 7) {
      pose1.covariance[i] = 0.1;
      pose2.covariance[i] = 0.2;
    }

    pose_cov_ops::compose(pose1, pose2, result);

    // Sum diagonals of input covariances
    double expected_cov_sum = 0.0;
    for (int i = 0; i < 36; i += 7) {
      expected_cov_sum += pose1.covariance[i] + pose2.covariance[i];
    }

    // Sum diagonals of output covariance
    double result_cov_sum = 0.0;
    for (int i = 0; i < 36; i += 7) {
      result_cov_sum += result.covariance[i];
    }

    RCLCPP_INFO(get_logger(), "\nCovariance Propagation Test:");
    RCLCPP_INFO(get_logger(), "  Expected covariance diagonal sum: %.3f", expected_cov_sum);
    RCLCPP_INFO(get_logger(), "  Result covariance diagonal sum:   %.3f", result_cov_sum);

    double tolerance = 0.001;
    if (std::abs(result_cov_sum - expected_cov_sum) < tolerance) {
      RCLCPP_INFO(get_logger(), "  Covariance sum matches expected value");
    } else {
      RCLCPP_WARN(get_logger(), "  Covariance sum mismatch! Got %.3f, expected %.3f",
                  result_cov_sum, expected_cov_sum);
    }

    // Print all diagonal elements for detailed debugging
    for (int i = 0; i < 36; i += 7) {
      RCLCPP_INFO(get_logger(),
        "  Cov[%d]: input1=%.3f, input2=%.3f, result=%.3f",
        i, pose1.covariance[i], pose2.covariance[i], result.covariance[i]);
    }
  }

  void test_timed_poses() {
    auto now = this->now();
    geometry_msgs::msg::PoseWithCovariance input_pose, result;
    input_pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(), "\nTime-based Pose Tests:");

    rclcpp::Time test_times[] = {
      now,
      now + rclcpp::Duration::from_seconds(1.5),
      now + rclcpp::Duration::from_seconds(20.0)
    };

    for (const auto &t : test_times) {
      bool success = interface_->compose_up(input_pose, "robot", t, result);
      RCLCPP_INFO(get_logger(), "Request at time %.2f: success=%d, X=%.3f",
                  t.seconds(), success, result.pose.position.x);
      if (!success) {
        RCLCPP_INFO(get_logger(), "  Time %.2f correctly rejected", t.seconds());
      }
    }
  }

  void test_identity_operations() {
    geometry_msgs::msg::PoseWithCovariance pose, result;
    pose.pose.orientation.w = 1.0;

    pose_cov_ops::compose(pose, pose, result);

    double dist = std::sqrt(
      std::pow(result.pose.position.x, 2) +
      std::pow(result.pose.position.y, 2) +
      std::pow(result.pose.position.z, 2)
    );

    RCLCPP_INFO(get_logger(), "\nIdentity Operation Test:");
    RCLCPP_INFO(get_logger(), "  Result distance from origin: %.6f", dist);

    if (dist < 1e-6) {
      RCLCPP_INFO(get_logger(), "  Identity test passed");
    } else {
      RCLCPP_ERROR(get_logger(), "  Identity test failed");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
