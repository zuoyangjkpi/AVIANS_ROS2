#!/usr/bin/env python3
"""
ROS2 Node for NMPC Drone Person Tracking
Adapted for ROS2 Jazzy and Python 3.12
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from typing import Optional
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

# ROS2 message types
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float64MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from neural_network_msgs.msg import NeuralNetworkDetectionArray

from .nmpc_controller import DroneNMPCController
from .config import nmpc_config

class NMPCTrackerNode(Node):
    """ROS2 Node for NMPC-based drone person tracking"""
    
    def __init__(self):
        super().__init__('nmpc_tracker_node')
        
        # Initialize NMPC controller
        self.controller = DroneNMPCController()
        
        # Node state
        self.drone_state_received = False
        self.person_detected = False
        self.last_person_detection_time = 0.0
        self.control_enabled = True
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize ROS2 interfaces
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()
        
        self.get_logger().info("NMPC Tracker Node initialized")
    
    def _init_parameters(self):
        """Initialize ROS2 parameters"""
        # Declare parameters with default values
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('person_timeout', 2.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('drone_frame', 'X3/base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_link')
        
        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.person_timeout = self.get_parameter('person_timeout').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.drone_frame = self.get_parameter('drone_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        self.get_logger().info(f"Control frequency: {self.control_frequency} Hz")
        self.get_logger().info(f"Person timeout: {self.person_timeout} seconds")
    
    def _init_publishers(self):
        """Initialize ROS2 publishers"""
        # QoS profile for control commands (reliable, low latency)
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS profile for visualization (best effort, higher depth)
        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Control command publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            nmpc_config.TOPIC_CONTROL_OUTPUT, 
            control_qos
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            Float64MultiArray,
            nmpc_config.TOPIC_STATUS,
            control_qos
        )
        
        # Visualization publishers
        if self.enable_visualization:
            self.trajectory_pub = self.create_publisher(
                MarkerArray,
                nmpc_config.TOPIC_TRAJECTORY_VIS,
                viz_qos
            )
            
            self.target_pub = self.create_publisher(
                Marker,
                nmpc_config.TOPIC_TARGET_VIS,
                viz_qos
            )
        
        self.get_logger().info("Publishers initialized")
    
    def _init_subscribers(self):
        """Initialize ROS2 subscribers"""
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Drone state subscriber (odometry)
        self.odom_sub = self.create_subscription(
            Odometry,
            nmpc_config.TOPIC_DRONE_STATE,
            self.drone_state_callback,
            sensor_qos
        )
        
        # Person detection subscriber
        self.detection_sub = self.create_subscription(
            NeuralNetworkDetectionArray,
            nmpc_config.TOPIC_PERSON_DETECTIONS,
            self.person_detection_callback,
            sensor_qos
        )
        
        # Control enable/disable subscriber
        self.enable_sub = self.create_subscription(
            Bool,
            '/nmpc/enable',
            self.enable_callback,
            sensor_qos
        )
        
        self.get_logger().info("Subscribers initialized")
    
    def _init_timers(self):
        """Initialize ROS2 timers"""
        # Main control loop timer
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(
            control_period,
            self.control_loop_callback
        )
        
        # Status publishing timer (lower frequency)
        self.status_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_status
        )
        
        self.get_logger().info(f"Control loop running at {self.control_frequency} Hz")
    
    def drone_state_callback(self, msg: Odometry):
        """Process drone state from odometry"""
        try:
            # Extract position
            position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            # Extract velocity
            velocity = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            
            # Extract orientation (quaternion to Euler)
            q = msg.pose.pose.orientation
            orientation = self._quaternion_to_euler(q.x, q.y, q.z, q.w)
            
            # Extract angular velocity
            angular_velocity = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            
            # Update controller state
            self.controller.set_drone_state(position, velocity, orientation, angular_velocity)
            self.drone_state_received = True
            
        except Exception as e:
            self.get_logger().error(f"Error processing drone state: {e}")
    
    def person_detection_callback(self, msg: NeuralNetworkDetectionArray):
        """Process person detection messages"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Find person detections (object_class = 1 for person)
            person_detections = []
            for detection in msg.detections:
                if detection.object_class == 1 and detection.detection_score > 0.5:
                    person_detections.append(detection)
            
            if person_detections:
                # Use the detection with highest confidence
                best_detection = max(person_detections, key=lambda d: d.detection_score)
                
                # Convert detection to 3D position
                person_position = self._detection_to_3d_position(best_detection)
                
                if person_position is not None:
                    # Estimate velocity (simple finite difference)
                    person_velocity = self._estimate_person_velocity(person_position, current_time)
                    
                    # Update controller
                    self.controller.set_person_detection(person_position, person_velocity)
                    self.person_detected = True
                    self.last_person_detection_time = current_time
                    
                    self.get_logger().debug(
                        f"Person detected at: {person_position}, confidence: {best_detection.detection_score:.2f}"
                    )
            
        except Exception as e:
            self.get_logger().error(f"Error processing person detection: {e}")
    
    def _detection_to_3d_position(self, detection) -> Optional[np.ndarray]:
        """Convert 2D detection to 3D world position"""
        try:
            # Get camera transform
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Camera position in world frame
            camera_pos = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Simple depth estimation based on detection size
            # This is a simplified approach - in practice, you might use stereo vision,
            # depth camera, or other methods for better depth estimation
            detection_height = abs(detection.ymax - detection.ymin)
            estimated_depth = self._estimate_depth_from_bbox_size(detection_height)
            
            # Convert pixel coordinates to world coordinates
            # This requires camera calibration parameters
            # For now, use a simplified projection model
            
            # Center of bounding box in normalized coordinates
            # Assuming detection coordinates are in pixels, convert to normalized [0,1] then to [-1,1]
            image_width = 640  # Assumed image width
            image_height = 480  # Assumed image height
            
            center_x_norm = ((detection.xmin + detection.xmax) / 2.0) / image_width
            center_y_norm = ((detection.ymin + detection.ymax) / 2.0) / image_height
            
            center_x = (center_x_norm - 0.5) * 2.0  # [-1, 1]
            center_y = (center_y_norm - 0.5) * 2.0  # [-1, 1]
            
            # Project to 3D (simplified)
            # Assume camera points forward with some downward tilt
            camera_direction = np.array([1.0, 0.0, -0.3])  # Forward and slightly down
            camera_direction = camera_direction / np.linalg.norm(camera_direction)
            
            # Estimate person position
            person_position = camera_pos + camera_direction * estimated_depth
            person_position[0] += center_x * estimated_depth * 0.5  # Lateral offset
            person_position[2] = 0.0  # Assume person is on ground
            
            return person_position
            
        except TransformException as e:
            self.get_logger().warn(f"Could not get camera transform: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error in 3D position estimation: {e}")
            return None
    
    def _estimate_depth_from_bbox_size(self, bbox_height: float) -> float:
        """Estimate depth based on bounding box size"""
        # Simple inverse relationship: larger bbox = closer person
        # This assumes average person height of 1.7m
        # bbox_height is now in pixels, normalize it first
        
        image_height = 480  # Assumed image height
        normalized_height = bbox_height / image_height
        
        if normalized_height > 0.01:  # Avoid division by zero
            # Empirical relationship - tune based on your camera setup
            estimated_depth = 0.3 / normalized_height  # meters
            return np.clip(estimated_depth, 2.0, 20.0)  # Reasonable range
        else:
            return 10.0  # Default distance
    
    def _estimate_person_velocity(self, position: np.ndarray, timestamp: float) -> np.ndarray:
        """Estimate person velocity using finite differences"""
        # Simple velocity estimation - in practice, you might use a Kalman filter
        if hasattr(self, '_last_person_position') and hasattr(self, '_last_person_time'):
            dt = timestamp - self._last_person_time
            if dt > 0.01:  # Avoid very small time differences
                velocity = (position - self._last_person_position) / dt
                # Smooth velocity estimate
                if hasattr(self, '_person_velocity_estimate'):
                    alpha = 0.3  # Smoothing factor
                    velocity = alpha * velocity + (1 - alpha) * self._person_velocity_estimate
                self._person_velocity_estimate = velocity
            else:
                velocity = getattr(self, '_person_velocity_estimate', np.zeros(3))
        else:
            velocity = np.zeros(3)
        
        # Store for next iteration
        self._last_person_position = position.copy()
        self._last_person_time = timestamp
        
        return velocity
    
    def enable_callback(self, msg: Bool):
        """Handle control enable/disable commands"""
        self.control_enabled = msg.data
        self.get_logger().info(f"Control {'enabled' if self.control_enabled else 'disabled'}")
    
    def control_loop_callback(self):
        """Main control loop"""
        if not self.control_enabled or not self.drone_state_received:
            return
        
        try:
            # Check if person detection is still valid
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (self.person_detected and 
                current_time - self.last_person_detection_time > self.person_timeout):
                self.person_detected = False
                self.get_logger().warn("Person detection timeout - switching to hover mode")
            
            # Run NMPC optimization
            optimal_control, info = self.controller.optimize()
            
            # Convert control to ROS2 Twist message
            cmd_msg = self._control_to_twist(optimal_control)
            
            # Publish control command
            self.cmd_vel_pub.publish(cmd_msg)
            
            # Publish visualization if enabled
            if self.enable_visualization:
                self._publish_visualization(info)
            
            # Log performance
            if info['iterations'] >= nmpc_config.MAX_ITERATIONS * 0.9:
                self.get_logger().warn(
                    f"NMPC optimization near iteration limit: {info['iterations']}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            # Publish hover command as safety fallback
            hover_cmd = Twist()
            self.cmd_vel_pub.publish(hover_cmd)
    
    def _control_to_twist(self, control: np.ndarray) -> Twist:
        """Convert NMPC control output to ROS2 Twist message"""
        cmd = Twist()
        
        # Extract control components
        thrust = control[0]
        roll_cmd = control[1]
        pitch_cmd = control[2]
        yaw_rate_cmd = control[3]
        
        # Convert to velocity commands (simplified)
        # This conversion depends on your specific drone controller
        
        # Vertical velocity from thrust
        hover_thrust = nmpc_config.DRONE_MASS * nmpc_config.GRAVITY
        thrust_normalized = (thrust - hover_thrust) / hover_thrust
        cmd.linear.z = thrust_normalized * 2.0  # Scale factor
        
        # Horizontal velocities from roll/pitch commands
        max_tilt = math.pi / 6  # 30 degrees max tilt
        cmd.linear.x = (pitch_cmd / max_tilt) * nmpc_config.DRONE_MAX_VELOCITY
        cmd.linear.y = -(roll_cmd / max_tilt) * nmpc_config.DRONE_MAX_VELOCITY  # Note: negative for correct direction
        
        # Yaw rate
        cmd.angular.z = yaw_rate_cmd
        
        # Apply velocity limits
        cmd.linear.x = np.clip(cmd.linear.x, -nmpc_config.DRONE_MAX_VELOCITY, nmpc_config.DRONE_MAX_VELOCITY)
        cmd.linear.y = np.clip(cmd.linear.y, -nmpc_config.DRONE_MAX_VELOCITY, nmpc_config.DRONE_MAX_VELOCITY)
        cmd.linear.z = np.clip(cmd.linear.z, -nmpc_config.DRONE_MAX_VELOCITY, nmpc_config.DRONE_MAX_VELOCITY)
        cmd.angular.z = np.clip(cmd.angular.z, -nmpc_config.DRONE_MAX_ANGULAR_VELOCITY, nmpc_config.DRONE_MAX_ANGULAR_VELOCITY)
        
        return cmd
    
    def _publish_visualization(self, info: dict):
        """Publish visualization markers"""
        try:
            # Publish trajectory
            if 'trajectory' in info:
                self._publish_trajectory_markers(info['trajectory'])
            
            # Publish target marker
            self._publish_target_marker()
            
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {e}")
    
    def _publish_trajectory_markers(self, trajectory):
        """Publish trajectory as marker array"""
        marker_array = MarkerArray()
        
        for i, state in enumerate(trajectory):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nmpc_trajectory"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(state[nmpc_config.STATE_X])
            marker.pose.position.y = float(state[nmpc_config.STATE_Y])
            marker.pose.position.z = float(state[nmpc_config.STATE_Z])
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color (fade along trajectory)
            alpha = 1.0 - (i / len(trajectory))
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = alpha
            
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            
            marker_array.markers.append(marker)
        
        self.trajectory_pub.publish(marker_array)
    
    def _publish_target_marker(self):
        """Publish target position marker"""
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nmpc_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(self.controller.target_position[0])
        marker.pose.position.y = float(self.controller.target_position[1])
        marker.pose.position.z = float(self.controller.target_position[2])
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        self.target_pub.publish(marker)
    
    def publish_status(self):
        """Publish controller status"""
        try:
            status = self.controller.get_status()
            
            msg = Float64MultiArray()
            msg.data = [
                float(status['person_detected']),
                status['tracking_distance'],
                status['optimization_time'],
                float(status['iterations_used']),
                status['cost_value']
            ]
            
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def _quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = NMPCTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

