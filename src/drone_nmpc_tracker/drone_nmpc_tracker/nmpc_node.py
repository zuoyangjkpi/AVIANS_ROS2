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

        # Vehicle references
        self.home_position: Optional[np.ndarray] = None
        self.home_yaw: float = 0.0
        self.last_tracking_yaw: float = 0.0
        self.last_known_position: Optional[np.ndarray] = None
        self.takeoff_target_altitude: Optional[float] = None

        # State machine management
        self.state: Optional[str] = None
        self.state_enter_time: float = 0.0
        self.takeoff_alt_reached_time: Optional[float] = None
        self.search_reference_position: Optional[np.ndarray] = None
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize ROS2 interfaces
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()
        self._switch_state('TAKEOFF')
        
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
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('takeoff_ascent_rate', 0.3)
        self.declare_parameter('takeoff_hold_duration', 10.0)
        self.declare_parameter('lost_target_hold_duration', 10.0)
        self.declare_parameter('altitude_tolerance', 0.05)
        self.declare_parameter('search_yaw_rate', 0.3)
        self.declare_parameter('tracking_phase_offset', 0.0)
        self.declare_parameter('tracking_height_offset', nmpc_config.TRACKING_HEIGHT_OFFSET)

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
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.takeoff_ascent_rate = self.get_parameter('takeoff_ascent_rate').value
        self.takeoff_hold_duration = self.get_parameter('takeoff_hold_duration').value
        self.lost_target_hold_duration = self.get_parameter('lost_target_hold_duration').value
        self.altitude_tolerance = max(0.01, self.get_parameter('altitude_tolerance').value)
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').value
        self.tracking_phase_offset = self.get_parameter('tracking_phase_offset').value
        self.tracking_height_offset = self.get_parameter('tracking_height_offset').value
        self.controller.set_tracking_height_offset(self.tracking_height_offset)

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
            if self.home_position is None:
                self.home_position = position.copy()
                self.home_yaw = orientation[2]
                self.takeoff_target_altitude = self.home_position[2] + self.takeoff_altitude
                self.last_tracking_yaw = self.home_yaw
            
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
            self._handle_person_detection(person_position, person_velocity)

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
            self._handle_person_detection(person_position, person_velocity)

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
    
    def _handle_person_detection(self, person_position: np.ndarray, person_velocity: np.ndarray):
        """Common logic when a person detection is received"""
        current_time = self._now()
        first_detection = not self.person_detected
        self.person_detected = True
        self.last_person_detection_time = current_time
        if first_detection:
            self.get_logger().info(f"✅ Person detected at {person_position}")
        if self.state != 'TRACK':
            self._switch_state('TRACK')

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
        if not self.control_enabled:
            return
    
    def control_loop_callback(self):
        """Main control loop"""
        current_time = self._now()

        if not self.control_enabled:
            self.get_logger().warn("Control not enabled")
            return

        if not self.drone_state_received:
            self.get_logger().warn("No drone state received - waiting for odometry data")
            return

        if self.person_detected and current_time - self.last_person_detection_time > self.person_timeout:
            self.person_detected = False
            self.get_logger().warn("Person detection timeout - entering hold state")
            if self.state == 'TRACK':
                self._enter_lost_hold()

        if self.state == 'TAKEOFF':
            self._step_takeoff(current_time)
            return

        if self.state == 'SEARCH':
            if self.person_detected:
                self._switch_state('TRACK')
            else:
                self._send_search_commands(current_time)
                return

        if self.state == 'LOST_HOLD':
            if self.person_detected:
                self._switch_state('TRACK')
            else:
                self._send_lost_hold_command()
                if current_time - self.state_enter_time >= self.lost_target_hold_duration:
                    self._switch_state('SEARCH')
                return

        if self.state == 'TRACK':
            if not self.person_detected:
                self._enter_lost_hold()
                return
            self._perform_tracking()
            return

        # Fallback: ensure we always have a valid control mode
        self._switch_state('SEARCH')
    
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

        self.last_tracking_yaw = desired_yaw

    def _send_search_commands(self, current_time: float):
        if self.search_reference_position is None:
            self.search_reference_position = self._get_current_position().copy()

        if self.takeoff_target_altitude is not None:
            target_altitude = self.takeoff_target_altitude
        elif self.home_position is not None:
            target_altitude = self.home_position[2] + self.takeoff_altitude
        else:
            target_altitude = self.takeoff_altitude

        waypoint = np.array([
            self.search_reference_position[0],
            self.search_reference_position[1],
            target_altitude
        ])
        self._publish_waypoint(waypoint)

        yaw_command = (self.home_yaw if self.home_position is not None else 0.0) + \
            self.search_yaw_rate * (current_time - self.state_enter_time)
        self._publish_attitude(yaw_command)

    def _get_current_position(self) -> np.ndarray:
        """获取当前无人机位置"""
        return self.controller.current_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1]

    def _get_current_yaw(self) -> float:
        return self.controller.current_state.data[nmpc_config.STATE_YAW]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_waypoint(self, position: np.ndarray):
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = self.world_frame
        waypoint_msg.pose.position.x = float(position[0])
        waypoint_msg.pose.position.y = float(position[1])
        waypoint_msg.pose.position.z = float(position[2])
        waypoint_msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(waypoint_msg)

    def _publish_attitude(self, yaw: float, roll: float = 0.0, pitch: float = 0.0):
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = self.drone_frame
        attitude_msg.vector.x = float(roll)
        attitude_msg.vector.y = float(pitch)
        attitude_msg.vector.z = float(yaw)
        self.attitude_pub.publish(attitude_msg)

    def _step_takeoff(self, current_time: float):
        if self.home_position is None or self.takeoff_target_altitude is None:
            return

        elapsed = max(0.0, current_time - self.state_enter_time)
        ramp_altitude = self.home_position[2] + self.takeoff_ascent_rate * elapsed
        target_altitude = min(self.takeoff_target_altitude, ramp_altitude)
        target_altitude = max(target_altitude, self.home_position[2])

        waypoint = np.array([
            self.home_position[0],
            self.home_position[1],
            target_altitude
        ])
        self._publish_waypoint(waypoint)
        self._publish_attitude(self.home_yaw)

        current_altitude = self._get_current_position()[2]
        if abs(current_altitude - self.takeoff_target_altitude) < self.altitude_tolerance:
            if self.takeoff_alt_reached_time is None:
                self.takeoff_alt_reached_time = current_time
            if not self.person_detected and (current_time - self.takeoff_alt_reached_time) >= self.takeoff_hold_duration:
                self._switch_state('SEARCH')
        else:
            self.takeoff_alt_reached_time = None

        if self.person_detected:
            self._switch_state('TRACK')

    def _enter_lost_hold(self):
        self.last_known_position = self._get_current_position().copy()
        self.last_tracking_yaw = self._get_current_yaw()
        self._switch_state('LOST_HOLD')

    def _switch_state(self, new_state: str):
        previous = self.state
        if previous == new_state:
            return
        self.state = new_state
        self.state_enter_time = self._now()

        if new_state == 'TAKEOFF':
            self.takeoff_alt_reached_time = None
        if new_state == 'SEARCH':
            current_position = self._get_current_position()
            self.search_reference_position = current_position.copy()
        else:
            self.search_reference_position = None
        if new_state == 'TRACK':
            self.controller.reset_phase(self.tracking_phase_offset)
            self.controller.set_tracking_height_offset(self.tracking_height_offset)
            self.last_tracking_yaw = self._get_current_yaw()
        if new_state == 'LOST_HOLD' and self.last_known_position is None:
            self.last_known_position = self._get_current_position().copy()
        if new_state != 'LOST_HOLD':
            self.last_known_position = None

        self.get_logger().info(f"状态切换: {previous} -> {new_state}")
    
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
