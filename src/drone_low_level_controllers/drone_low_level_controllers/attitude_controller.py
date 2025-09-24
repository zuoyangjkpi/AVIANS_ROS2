#!/usr/bin/env python3
"""Attitude Controller Plugin for Drone Low-Level Control"""

import logging
from logging.handlers import RotatingFileHandler
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool


LOG_PATH = '/tmp/drone_low_level_controllers.log'


def _init_file_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=2)
        handler.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger

class AttitudeController(Node):
    def __init__(self):
        super().__init__('attitude_controller')

        # Parameters
        self.declare_parameter('control_frequency', 100.0)  # Higher frequency for attitude control
        self.declare_parameter('attitude_tolerance', 0.1)  # radians

        # PID gains for attitude control (similar to PX4 v1.16.0)
        self.declare_parameter('kp_roll', 4.0)
        self.declare_parameter('ki_roll', 0.0)
        self.declare_parameter('kd_roll', 0.2)
        self.declare_parameter('kp_pitch', 4.0)
        self.declare_parameter('ki_pitch', 0.0)
        self.declare_parameter('kd_pitch', 0.2)
        self.declare_parameter('kp_yaw', 3.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.1)

        # Max angular velocities (rad/s) (similar to PX4 constraints)
        self.declare_parameter('max_angular_velocity_roll', 3.0)
        self.declare_parameter('max_angular_velocity_pitch', 3.0)
        self.declare_parameter('max_angular_velocity_yaw', 2.0)

        # Get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        self.attitude_tolerance = self.get_parameter('attitude_tolerance').value

        # PID gains
        self.kp_roll = self.get_parameter('kp_roll').value
        self.ki_roll = self.get_parameter('ki_roll').value
        self.kd_roll = self.get_parameter('kd_roll').value
        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.ki_pitch = self.get_parameter('ki_pitch').value
        self.kd_pitch = self.get_parameter('kd_pitch').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.ki_yaw = self.get_parameter('ki_yaw').value
        self.kd_yaw = self.get_parameter('kd_yaw').value

        self.max_angular_velocity = np.array([
            self.get_parameter('max_angular_velocity_roll').value,
            self.get_parameter('max_angular_velocity_pitch').value,
            self.get_parameter('max_angular_velocity_yaw').value
        ])

        # State variables
        self.current_attitude = None  # [roll, pitch, yaw] (yaw unwrapped)
        self.current_angular_velocity = None
        self.target_attitude = None  # [roll, pitch, yaw]
        self.last_target_attitude = None
        self.last_command_time = None
        self.unwrapped_yaw = None
        self.last_yaw_measurement = None
        self.controller_active = False

        # PID state
        self.attitude_error_integral = np.zeros(3)
        self.attitude_error_previous = np.zeros(3)
        self.last_control_time = None

        # File logger for external monitoring
        self.file_logger = _init_file_logger('attitude_controller')
        self._log_counter = 0

        # Publishers
        self.angular_velocity_setpoint_pub = self.create_publisher(
            Vector3Stamped, '/drone/control/angular_velocity_setpoint', 10)
        self.attitude_reached_pub = self.create_publisher(
            Bool, '/drone/control/attitude_reached', 10)

        # Subscribers
        self.attitude_command_sub = self.create_subscription(
            Vector3Stamped, '/drone/control/attitude_command',
            self.attitude_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/drone/control/attitude_enable',
            self.enable_callback, 10)

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(f'Attitude controller initialized at {self.control_frequency} Hz')

    def attitude_callback(self, msg: Vector3Stamped):
        """Receive attitude command from high-level controller"""
        new_target = np.array([
            msg.vector.x,  # roll
            msg.vector.y,  # pitch
            msg.vector.z   # yaw
        ])

        significant_change = True
        if self.last_target_attitude is not None:
            delta = np.abs(new_target - self.last_target_attitude)
            significant_change = bool(np.max(delta) > math.radians(2.0))

        if self.last_target_attitude is not None:
            yaw_delta = self.normalize_angle(new_target[2] - self.last_target_attitude[2])
            new_target[2] = self.last_target_attitude[2] + yaw_delta
        self.target_attitude = new_target
        self.last_target_attitude = new_target.copy()
        self.last_command_time = self.get_clock().now()

        if significant_change:
            # Reset PID history only when command jumps notably
            self.attitude_error_integral = np.zeros(3)
            self.attitude_error_previous = np.zeros(3)

        self.get_logger().debug(f'New attitude command: roll={math.degrees(self.target_attitude[0]):.1f}°, '
                               f'pitch={math.degrees(self.target_attitude[1]):.1f}°, '
                               f'yaw={math.degrees(self.target_attitude[2]):.1f}°')

    def odometry_callback(self, msg: Odometry):
        """Update current drone attitude and angular velocity"""
        # Extract quaternion
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        # Convert to Euler angles (roll, pitch, yaw)
        rotation = Rotation.from_quat(quat)
        euler = rotation.as_euler('xyz', degrees=False)

        wrapped_yaw = euler[2]
        if self.unwrapped_yaw is None:
            self.unwrapped_yaw = wrapped_yaw
        else:
            yaw_delta = self.normalize_angle(wrapped_yaw - self.last_yaw_measurement)
            self.unwrapped_yaw += yaw_delta

        self.last_yaw_measurement = wrapped_yaw
        self.current_attitude = np.array([euler[0], euler[1], self.unwrapped_yaw])

        # Extract angular velocity
        self.current_angular_velocity = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

    def enable_callback(self, msg: Bool):
        """Enable/disable attitude controller"""
        self.controller_active = msg.data
        if self.controller_active:
            self.get_logger().info('Attitude controller ENABLED')
            self.file_logger.info('controller_enabled')
        else:
            self.get_logger().info('Attitude controller DISABLED')
            self.file_logger.info('controller_disabled -> zero angular rate')

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        """Main control loop for attitude control"""
        if not self.controller_active:
            return

        if (self.current_attitude is None or
            self.target_attitude is None):
            return

        current_time = self.get_clock().now()

        # Calculate time step
        if self.last_control_time is not None:
            dt = (current_time - self.last_control_time).nanoseconds / 1e9
        else:
            dt = 1.0 / self.control_frequency
        self.last_control_time = current_time

        # Attitude error with angle normalization
        attitude_error = np.zeros(3)
        for i in range(3):
            attitude_error[i] = self.normalize_angle(self.target_attitude[i] - self.current_attitude[i])

        # Check if attitude is reached
        attitude_magnitude = np.linalg.norm(attitude_error)
        command_recent = False
        if self.last_command_time is not None:
            dt_command = (current_time - self.last_command_time).nanoseconds / 1e9
            command_recent = dt_command < 0.15

        if attitude_magnitude < self.attitude_tolerance and not command_recent:
            attitude_reached_msg = Bool()
            attitude_reached_msg.data = True
            self.attitude_reached_pub.publish(attitude_reached_msg)

        # PID control
        # Integral term (with windup prevention)
        self.attitude_error_integral += attitude_error * dt
        integral_deadband = np.array([math.radians(0.5), math.radians(0.5), math.radians(0.5)])
        for i in range(3):
            if abs(attitude_error[i]) < integral_deadband[i]:
                self.attitude_error_integral[i] = 0.0
        for i in range(3):
            if attitude_error[i] * self.attitude_error_previous[i] <= 0.0:
                self.attitude_error_integral[i] = 0.0
        max_integral = 2.0  # Prevent windup
        self.attitude_error_integral = np.clip(
            self.attitude_error_integral, -max_integral, max_integral)

        # Derivative term
        error_derivative = np.zeros(3)
        if dt > 0:
            error_derivative = (attitude_error - self.attitude_error_previous) / dt

        # PID gains
        kp = np.array([self.kp_roll, self.kp_pitch, self.kp_yaw])
        ki = np.array([self.ki_roll, self.ki_pitch, self.ki_yaw])
        kd = np.array([self.kd_roll, self.kd_pitch, self.kd_yaw])

        # Calculate angular velocity command
        angular_velocity_command = (kp * attitude_error +
                                   ki * self.attitude_error_integral +
                                   kd * error_derivative)

        # Apply angular velocity limits
        angular_velocity_command = np.clip(
            angular_velocity_command, -self.max_angular_velocity, self.max_angular_velocity)

        # Publish angular velocity setpoint
        angular_velocity_cmd = Vector3Stamped()
        angular_velocity_cmd.header.stamp = current_time.to_msg()
        angular_velocity_cmd.header.frame_id = 'X3/base_link'
        angular_velocity_cmd.vector.x = angular_velocity_command[0]  # roll rate
        angular_velocity_cmd.vector.y = angular_velocity_command[1]  # pitch rate
        angular_velocity_cmd.vector.z = angular_velocity_command[2]  # yaw rate

        self.angular_velocity_setpoint_pub.publish(angular_velocity_cmd)

        self._log_counter += 1
        if self._log_counter >= 5:
            self.file_logger.info(
                'attitude_ctrl ang_cmd=[%.3f, %.3f, %.3f] err=[%.3f, %.3f, %.3f]',
                angular_velocity_command[0], angular_velocity_command[1], angular_velocity_command[2],
                attitude_error[0], attitude_error[1], attitude_error[2]
            )
            self._log_counter = 0

        # Update previous error
        self.attitude_error_previous = attitude_error.copy()

        # Debug logging
        self.get_logger().debug(
            f'Attitude control: error=[{math.degrees(attitude_error[0]):.1f}°, '
            f'{math.degrees(attitude_error[1]):.1f}°, {math.degrees(attitude_error[2]):.1f}°], '
            f'cmd_ang_vel=[{angular_velocity_command[0]:.2f}, {angular_velocity_command[1]:.2f}, {angular_velocity_command[2]:.2f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    attitude_controller = AttitudeController()

    try:
        rclpy.spin(attitude_controller)
    except KeyboardInterrupt:
        pass

    attitude_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
