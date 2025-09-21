#!/usr/bin/env python3
"""
Waypoint Controller Plugin for Drone Low-Level Control

This controller accepts waypoint commands from high-level controllers (like NMPC)
and publishes position setpoints for the attitude controller.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        # Parameters
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('position_tolerance', 0.2)  # meters
        self.declare_parameter('velocity_tolerance', 0.1)  # m/s
        self.declare_parameter('waypoint_timeout', 0.5)    # seconds

        # PID gains for position control (similar to PX4 v1.16.0)
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('ki_xy', 0.05)
        self.declare_parameter('kd_xy', 0.01)
        self.declare_parameter('kp_z', 1.0)
        self.declare_parameter('ki_z', 0.05)
        self.declare_parameter('kd_z', 0.01)

        # Max velocities (similar to PX4 constraints)
        self.declare_parameter('max_velocity_xy', 3.0)
        self.declare_parameter('max_velocity_z', 1.5)

        # Get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.velocity_tolerance = self.get_parameter('velocity_tolerance').value
        self.waypoint_timeout = float(self.get_parameter('waypoint_timeout').value)

        # PID gains
        self.kp_xy = self.get_parameter('kp_xy').value
        self.ki_xy = self.get_parameter('ki_xy').value
        self.kd_xy = self.get_parameter('kd_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value

        self.max_velocity_xy = self.get_parameter('max_velocity_xy').value
        self.max_velocity_z = self.get_parameter('max_velocity_z').value

        # State variables
        self.current_pose = None
        self.current_velocity = None
        self.target_waypoint = None
        self.controller_active = False

        # PID state
        self.position_error_integral = np.zeros(3)
        self.position_error_previous = np.zeros(3)
        self.last_control_time = None
        self.last_waypoint_time = None

        # Publishers
        self.velocity_setpoint_pub = self.create_publisher(
            TwistStamped, '/drone/control/velocity_setpoint', 10)
        self.waypoint_reached_pub = self.create_publisher(
            Bool, '/drone/control/waypoint_reached', 10)

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/drone/control/waypoint_command',
            self.waypoint_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/drone/control/waypoint_enable',
            self.enable_callback, 10)

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(f'Waypoint controller initialized at {self.control_frequency} Hz')

    def waypoint_callback(self, msg: PoseStamped):
        """Receive waypoint command from high-level controller"""
        self.target_waypoint = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.last_waypoint_time = self.get_clock().now()

        # Reset PID state for new waypoint
        self.position_error_integral = np.zeros(3)
        self.position_error_previous = np.zeros(3)

        self.get_logger().info(f'New waypoint received: {self.target_waypoint}')

    def odometry_callback(self, msg: Odometry):
        """Update current drone state"""
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

    def enable_callback(self, msg: Bool):
        """Enable/disable waypoint controller"""
        self.controller_active = msg.data
        if self.controller_active:
            self.get_logger().info('Waypoint controller ENABLED')
        else:
            self.get_logger().info('Waypoint controller DISABLED')
            self.target_waypoint = None
            self.last_waypoint_time = None
            self.position_error_integral = np.zeros(3)
            self.position_error_previous = np.zeros(3)
            self.last_control_time = None

    def control_loop(self):
        """Main control loop for waypoint following"""
        current_time = self.get_clock().now()

        if not self.controller_active:
            return

        if self.current_pose is None:
            return

        if self.target_waypoint is None:
            return

        if (self.last_waypoint_time is not None and
            (current_time - self.last_waypoint_time).nanoseconds / 1e9 > self.waypoint_timeout):
            zero_cmd = TwistStamped()
            zero_cmd.header.stamp = current_time.to_msg()
            zero_cmd.header.frame_id = 'world'
            self.velocity_setpoint_pub.publish(zero_cmd)
            self.target_waypoint = None
            self.last_waypoint_time = None
            self.position_error_integral = np.zeros(3)
            self.position_error_previous = np.zeros(3)
            self.last_control_time = None
            return

        if self.current_velocity is None:
            self.current_velocity = np.zeros(3)

        # Calculate time step
        if self.last_control_time is not None:
            dt = (current_time - self.last_control_time).nanoseconds / 1e9
        else:
            dt = 1.0 / self.control_frequency
        self.last_control_time = current_time

        # Position error
        position_error = self.target_waypoint - self.current_pose

        # Check if waypoint is reached
        position_magnitude = np.linalg.norm(position_error)
        velocity_magnitude = np.linalg.norm(self.current_velocity) if self.current_velocity is not None else 0.0

        if (position_magnitude < self.position_tolerance and
            velocity_magnitude < self.velocity_tolerance):
            # Waypoint reached
            waypoint_reached_msg = Bool()
            waypoint_reached_msg.data = True
            self.waypoint_reached_pub.publish(waypoint_reached_msg)

            # Stop motion
            velocity_cmd = TwistStamped()
            velocity_cmd.header.stamp = current_time.to_msg()
            velocity_cmd.header.frame_id = 'world'
            self.velocity_setpoint_pub.publish(velocity_cmd)
            return

        # PID control
        # Integral term (with windup prevention)
        self.position_error_integral += position_error * dt
        max_integral = 5.0  # Prevent windup
        self.position_error_integral = np.clip(
            self.position_error_integral, -max_integral, max_integral)

        # Derivative term
        error_derivative = np.zeros(3)
        if dt > 0:
            error_derivative = (position_error - self.position_error_previous) / dt

        # PID gains (different for xy and z)
        kp = np.array([self.kp_xy, self.kp_xy, self.kp_z])
        ki = np.array([self.ki_xy, self.ki_xy, self.ki_z])
        kd = np.array([self.kd_xy, self.kd_xy, self.kd_z])

        # Calculate velocity command
        velocity_command = (kp * position_error +
                           ki * self.position_error_integral +
                           kd * error_derivative)

        # Apply velocity limits
        max_vel = np.array([self.max_velocity_xy, self.max_velocity_xy, self.max_velocity_z])
        velocity_command = np.clip(velocity_command, -max_vel, max_vel)

        # Publish velocity setpoint
        velocity_cmd = TwistStamped()
        velocity_cmd.header.stamp = current_time.to_msg()
        velocity_cmd.header.frame_id = 'world'
        velocity_cmd.twist.linear.x = velocity_command[0]
        velocity_cmd.twist.linear.y = velocity_command[1]
        velocity_cmd.twist.linear.z = velocity_command[2]

        self.velocity_setpoint_pub.publish(velocity_cmd)

        # Update previous error
        self.position_error_previous = position_error.copy()

        # Debug logging
        self.get_logger().debug(
            f'Waypoint control: error={position_magnitude:.3f}m, '
            f'cmd_vel=[{velocity_command[0]:.2f}, {velocity_command[1]:.2f}, {velocity_command[2]:.2f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    waypoint_controller = WaypointController()

    try:
        rclpy.spin(waypoint_controller)
    except KeyboardInterrupt:
        pass

    waypoint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
