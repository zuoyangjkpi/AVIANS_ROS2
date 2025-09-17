#!/usr/bin/env python3
"""
ROS2 Node for NMPC Drone Person Tracking
Adapted for ROS2 Jazzy and Python 3.12
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import numpy as np
import math
from typing import Optional
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

# ROS2 message types
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped, TwistStamped, PoseWithCovarianceStamped
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

        # Search mode state
        self.search_target_waypoint = None  # Target waypoint for search mode
        self.search_target_attitude = None  # Target attitude for search mode
        self.search_mode_active = False
        self.search_start_time = None
        
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
        self.declare_parameter('camera_frame', 'machine_1_camera_link')
        # Align with projection_model output topic started in the integration script
        self.declare_parameter('projected_detection_topic', '/person_detections/world_frame')
        self.declare_parameter('use_actor_groundtruth', True)
        self.declare_parameter('actor_pose_topic', '/actor/walking_person/pose')

        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.person_timeout = self.get_parameter('person_timeout').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.drone_frame = self.get_parameter('drone_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.projected_detection_topic = self.get_parameter('projected_detection_topic').value
        self.use_actor_groundtruth = self.get_parameter('use_actor_groundtruth').value
        self.actor_pose_topic = self.get_parameter('actor_pose_topic').value

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
        
        # Control command publishers for low-level controllers
        self.waypoint_pub = self.create_publisher(
            PoseStamped, 
            '/drone/control/waypoint_command', 
            control_qos
        )
        
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, 
            '/drone/control/attitude_command', 
            control_qos
        )
        
        self.velocity_pub = self.create_publisher(
            TwistStamped, 
            '/drone/control/velocity_setpoint', 
            control_qos
        )
        
        # Control enable/disable publishers
        self.waypoint_enable_pub = self.create_publisher(
            Bool,
            '/drone/control/waypoint_enable',
            control_qos
        )
        
        self.attitude_enable_pub = self.create_publisher(
            Bool,
            '/drone/control/attitude_enable',
            control_qos
        )
        
        self.velocity_enable_pub = self.create_publisher(
            Bool,
            '/drone/control/velocity_enable',
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
        # Use ROS2 standard sensor QoS preset
        sensor_qos = qos_profile_sensor_data
        
        # Drone state subscriber (odometry)
        self.odom_sub = self.create_subscription(
            Odometry,
            nmpc_config.TOPIC_DRONE_STATE,
            self.drone_state_callback,
            sensor_qos
        )
        
        # Person detection subscriber (using projected detections from projection_model)
        self.detection_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.projected_detection_topic,
            self.projected_detection_callback,
            sensor_qos
        )

        # Control enable/disable subscriber
        self.enable_sub = self.create_subscription(
            Bool,
            '/nmpc/enable',
            self.enable_callback,
            sensor_qos  # Use sensor_qos for consistency
        )

        if self.use_actor_groundtruth:
            self.actor_pose_sub = self.create_subscription(
                PoseStamped,
                self.actor_pose_topic,
                self.actor_pose_callback,
                sensor_qos
            )
            self.get_logger().info(
                f"Actor ground-truth tracking enabled via {self.actor_pose_topic}"
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
            
            # ✅ 添加调试信息
            self.get_logger().info(f"Drone state updated: pos={position}, vel={velocity}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing drone state: {e}")
    
    def projected_detection_callback(self, msg: PoseWithCovarianceStamped):
        """Process projected person detection messages from projection_model"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Extract person position from projected detection
            person_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            # Estimate velocity (simple finite difference)
            person_velocity = self._estimate_person_velocity(person_position, current_time)

            # Update controller
            self.controller.set_person_detection(person_position, person_velocity)
            self.person_detected = True
            self.last_person_detection_time = current_time

            self.get_logger().info(
                f"✅ Person tracking ENABLED: pos={person_position}"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing projected person detection: {e}")

    def actor_pose_callback(self, msg: PoseStamped):
        """Use Gazebo actor ground-truth pose as person detection"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            person_position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])

            person_velocity = self._estimate_person_velocity(person_position, current_time)

            self.controller.set_person_detection(person_position, person_velocity)
            was_tracking = self.person_detected
            self.person_detected = True
            self.last_person_detection_time = current_time

            if not was_tracking:
                self.get_logger().info(
                    f"✅ Ground-truth actor tracking enabled: pos={person_position}"
                )

        except Exception as e:
            self.get_logger().error(f"Error processing actor pose: {e}")
    
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
        
        # Enable/disable low-level controllers
        enable_msg = Bool()
        enable_msg.data = self.control_enabled
        
        self.waypoint_enable_pub.publish(enable_msg)
        self.attitude_enable_pub.publish(enable_msg)
        self.velocity_enable_pub.publish(enable_msg)
    
    def control_loop_callback(self):
        """Main control loop"""
        # ✅ 添加调试信息
        self.get_logger().info(f"Control enabled: {self.control_enabled}, Drone state received: {self.drone_state_received}")
        
        if not self.control_enabled:
            self.get_logger().warn("Control not enabled")
            return
            
        if not self.drone_state_received:
            self.get_logger().warn("No drone state received - waiting for odometry data")
            return
        
        try:
            # Check if person detection is still valid
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (self.person_detected and 
                current_time - self.last_person_detection_time > self.person_timeout):
                self.person_detected = False
                self.get_logger().warn("Person detection timeout - switching to search mode")
            
            # ✅ 搜索模式：等待底层控制插件实现
            if not self.person_detected:
                if not self.search_mode_active:
                    self._initialize_search_mode()
                    self.search_start_time = current_time

                # Send search commands to low-level controllers
                self._send_search_commands(current_time)
                self.get_logger().info(f"Search mode: Sending commands to low-level controllers")
                return
            else:
                # 如果检测到人员，退出搜索模式
                if self.search_mode_active:
                    self.search_mode_active = False
                    self.get_logger().info("Person detected - exiting search mode")
            
            # Run NMPC optimization (只在有人员检测时)
            optimal_control, info = self.controller.optimize()
            
            # Convert control to low-level controller commands
            self._send_tracking_commands(optimal_control)
            
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
            self._send_hover_commands()
    
    def _send_tracking_commands(self, control: np.ndarray):
        """Send tracking commands to low-level controllers"""
        # Extract target position from controller
        target_position = self.controller.target_position
        
        # Create and publish waypoint command
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = self.world_frame
        waypoint_msg.pose.position.x = float(target_position[0])
        waypoint_msg.pose.position.y = float(target_position[1])
        waypoint_msg.pose.position.z = float(target_position[2])
        waypoint_msg.pose.orientation.w = 1.0
        
        self.waypoint_pub.publish(waypoint_msg)
        
        # Calculate desired attitude (face the person)
        current_pos = self.controller.current_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1]
        person_pos = self.controller.person_position
        
        to_person = person_pos - current_pos
        desired_yaw = math.atan2(to_person[1], to_person[0])
        
        # Create and publish attitude command
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = self.drone_frame
        attitude_msg.vector.x = 0.0  # roll
        attitude_msg.vector.y = 0.0  # pitch
        attitude_msg.vector.z = desired_yaw  # yaw
        
        self.attitude_pub.publish(attitude_msg)
    
    def _send_hover_commands(self):
        """Send hover commands as safety fallback"""
        # Get current position as target
        current_pos = self.controller.current_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1]
        
        # Publish current position as waypoint
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = self.world_frame
        waypoint_msg.pose.position.x = float(current_pos[0])
        waypoint_msg.pose.position.y = float(current_pos[1])
        waypoint_msg.pose.position.z = float(current_pos[2])
        waypoint_msg.pose.orientation.w = 1.0
        
        self.waypoint_pub.publish(waypoint_msg)
        
        # Publish level attitude
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = self.drone_frame
        attitude_msg.vector.x = 0.0  # roll
        attitude_msg.vector.y = 0.0  # pitch
        attitude_msg.vector.z = 0.0  # yaw
        
        self.attitude_pub.publish(attitude_msg)
    
    def _send_search_commands(self, current_time: float):
        """Send search mode commands to low-level controllers"""
        if self.search_target_waypoint is not None:
            # Publish waypoint command (保持当前位置的XY坐标，Z坐标为2米)
            waypoint_msg = PoseStamped()
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()
            waypoint_msg.header.frame_id = self.world_frame
            waypoint_msg.pose.position.x = float(self.search_target_waypoint[0])
            waypoint_msg.pose.position.y = float(self.search_target_waypoint[1])
            waypoint_msg.pose.position.z = float(self.search_target_waypoint[2])  # 2米高度
            waypoint_msg.pose.orientation.w = 1.0
            
            self.waypoint_pub.publish(waypoint_msg)
        
        if self.search_target_attitude is not None:
            # Update yaw for rotation (0.3 rad/s 匀速转动)
            elapsed_time = current_time - self.search_start_time if self.search_start_time else 0.0
            self.search_target_attitude[2] = (elapsed_time * 0.3) % (2 * math.pi)  # yaw
            
            # Publish attitude command (roll和pitch为0，yaw匀速转动)
            attitude_msg = Vector3Stamped()
            attitude_msg.header.stamp = self.get_clock().now().to_msg()
            attitude_msg.header.frame_id = self.drone_frame
            attitude_msg.vector.x = float(self.search_target_attitude[0])  # roll = 0
            attitude_msg.vector.y = float(self.search_target_attitude[1])  # pitch = 0
            attitude_msg.vector.z = float(self.search_target_attitude[2])  # yaw = 匀速转动
            
            self.attitude_pub.publish(attitude_msg)
    
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
    
    def _initialize_search_mode(self):
        """初始化搜索模式"""
        current_pos = self._get_current_position()
        # 设置搜索目标：当前X,Y位置，高度2米，姿态为水平+旋转
        self.search_target_waypoint = np.array([current_pos[0], current_pos[1], 2.0])
        self.search_target_attitude = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] - yaw will rotate
        self.search_mode_active = True

        self.get_logger().info(f"Search mode initialized - target waypoint: {self.search_target_waypoint}")

    def _get_current_position(self) -> np.ndarray:
        """获取当前无人机位置"""
        return self.controller.current_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1]

    def _generate_simple_search_command(self, current_time: float) -> Twist:
        """临时简单搜索命令（等待底层控制插件实现）"""
        cmd = Twist()

        # 简单的上升和旋转
        current_pos = self._get_current_position()
        target_height = 2.0

        if current_pos[2] < target_height - 0.1:
            cmd.linear.z = 0.5  # 上升到2米
        else:
            cmd.linear.z = 0.0
            cmd.angular.z = 0.3  # 旋转搜索

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
