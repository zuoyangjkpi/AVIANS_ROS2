#!/usr/bin/env python3
"""
Velocity Controller Plugin for Drone Low-Level Control

This controller accepts velocity and angular velocity commands from other controllers
and publishes final control commands to /X3/cmd_vel for Gazebo execution.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Parameters
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('velocity_smoothing', 0.3)  # Smoothing factor (0.0 = no smoothing, 1.0 = full smoothing)

        # Velocity and angular velocity limits (similar to PX4 constraints)
        self.declare_parameter('max_linear_velocity_xy', 3.0)
        self.declare_parameter('max_linear_velocity_z', 2.0)
        self.declare_parameter('max_angular_velocity_xy', 3.0)  # roll/pitch rates
        self.declare_parameter('max_angular_velocity_z', 2.0)   # yaw rate

        # PID gains for velocity control (similar to PX4 v1.16.0)
        self.declare_parameter('kp_vx', 0.5)
        self.declare_parameter('kp_vy', 0.5)
        self.declare_parameter('kp_vz', 0.5)
        self.declare_parameter('kp_wx', 0.3)
        self.declare_parameter('kp_wy', 0.3)
        self.declare_parameter('kp_wz', 0.3)

        # Get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        self.velocity_smoothing = self.get_parameter('velocity_smoothing').value

        self.max_linear_velocity_xy = self.get_parameter('max_linear_velocity_xy').value
        self.max_linear_velocity_z = self.get_parameter('max_linear_velocity_z').value
        self.max_angular_velocity_xy = self.get_parameter('max_angular_velocity_xy').value
        self.max_angular_velocity_z = self.get_parameter('max_angular_velocity_z').value

        # PID gains
        self.kp_vx = self.get_parameter('kp_vx').value
        self.kp_vy = self.get_parameter('kp_vy').value
        self.kp_vz = self.get_parameter('kp_vz').value
        self.kp_wx = self.get_parameter('kp_wx').value
        self.kp_wy = self.get_parameter('kp_wy').value
        self.kp_wz = self.get_parameter('kp_wz').value

        # State variables
        self.current_velocity = None
        self.current_angular_velocity = None
        self.target_velocity = np.zeros(3)  # [vx, vy, vz]
        self.target_angular_velocity = np.zeros(3)  # [wx, wy, wz]
        self.smoothed_velocity = np.zeros(3)
        self.smoothed_angular_velocity = np.zeros(3)
        self.controller_active = False

        # Command sources tracking
        self.velocity_command_active = False
        self.angular_velocity_command_active = False
        self.last_velocity_command_time = None
        self.last_angular_velocity_command_time = None
        self.command_timeout = 0.5  # seconds

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/X3/cmd_vel', 10)

        # Subscribers
        self.velocity_setpoint_sub = self.create_subscription(
            TwistStamped, '/drone/control/velocity_setpoint',
            self.velocity_setpoint_callback, 10)
        self.angular_velocity_setpoint_sub = self.create_subscription(
            Vector3Stamped, '/drone/control/angular_velocity_setpoint',
            self.angular_velocity_setpoint_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/drone/control/velocity_enable',
            self.enable_callback, 10)

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(f'Velocity controller initialized at {self.control_frequency} Hz')

    def velocity_setpoint_callback(self, msg: TwistStamped):
        """Receive velocity setpoint from waypoint controller"""
        self.target_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        self.velocity_command_active = True
        self.last_velocity_command_time = self.get_clock().now()
        self.smoothed_velocity = self.target_velocity.copy()

        self.get_logger().debug(f'Velocity setpoint: [{self.target_velocity[0]:.2f}, '
                               f'{self.target_velocity[1]:.2f}, {self.target_velocity[2]:.2f}]')

    def angular_velocity_setpoint_callback(self, msg: Vector3Stamped):
        """Receive angular velocity setpoint from attitude controller"""
        self.target_angular_velocity = np.array([
            msg.vector.x,
            msg.vector.y,
            msg.vector.z
        ])
        self.angular_velocity_command_active = True
        self.last_angular_velocity_command_time = self.get_clock().now()

        self.get_logger().debug(f'Angular velocity setpoint: [{self.target_angular_velocity[0]:.2f}, '
                               f'{self.target_angular_velocity[1]:.2f}, {self.target_angular_velocity[2]:.2f}]')

    def odometry_callback(self, msg: Odometry):
        """Update current drone velocities"""
        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        self.current_angular_velocity = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

    def enable_callback(self, msg: Bool):
        """Enable/disable velocity controller"""
        self.controller_active = msg.data
        if self.controller_active:
            self.get_logger().info('Velocity controller ENABLED')
        else:
            self.get_logger().info('Velocity controller DISABLED')
            # Send zero command when disabled
            self._send_zero_command()

    def _send_zero_command(self):
        """Send zero velocity command to stop the drone"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _check_command_timeouts(self):
        """Check if commands have timed out and reset them if necessary"""
        current_time = self.get_clock().now()

        # Check velocity command timeout
        if (self.velocity_command_active and
            self.last_velocity_command_time is not None):
            time_diff = (current_time - self.last_velocity_command_time).nanoseconds / 1e9
            if time_diff > self.command_timeout:
                self.velocity_command_active = False
                self.target_velocity = np.zeros(3)
                self.smoothed_velocity = np.zeros(3)
                self.get_logger().debug('Velocity command timed out')

        # Check angular velocity command timeout
        if (self.angular_velocity_command_active and
            self.last_angular_velocity_command_time is not None):
            time_diff = (current_time - self.last_angular_velocity_command_time).nanoseconds / 1e9
            if time_diff > self.command_timeout:
                self.angular_velocity_command_active = False
                self.target_angular_velocity = np.zeros(3)
                self.smoothed_angular_velocity = np.zeros(3)
                self.get_logger().debug('Angular velocity command timed out')

    def control_loop(self):
        """Main control loop for velocity control"""
        if not self.controller_active:
            return

        # Check for command timeouts
        self._check_command_timeouts()

        if not self.velocity_command_active and not self.angular_velocity_command_active:
            self.smoothed_velocity = np.zeros(3)
            self.smoothed_angular_velocity = np.zeros(3)
            self._send_zero_command()
            return

        # Apply velocity smoothing
        alpha = self.velocity_smoothing
        self.smoothed_velocity = (alpha * self.smoothed_velocity +
                                 (1 - alpha) * self.target_velocity)
        self.smoothed_angular_velocity = (alpha * self.smoothed_angular_velocity +
                                         (1 - alpha) * self.target_angular_velocity)

        # Calculate velocity error
        velocity_error = np.zeros(3)
        angular_velocity_error = np.zeros(3)
        
        if self.current_velocity is not None:
            velocity_error = self.smoothed_velocity - self.current_velocity
            
        if self.current_angular_velocity is not None:
            angular_velocity_error = self.smoothed_angular_velocity - self.current_angular_velocity

        # PID control for velocity
        kp_v = np.array([self.kp_vx, self.kp_vy, self.kp_vz])
        acceleration_command = kp_v * velocity_error

        # PID control for angular velocity
        kp_w = np.array([self.kp_wx, self.kp_wy, self.kp_wz])
        torque_command = kp_w * angular_velocity_error

        # Mix control commands
        cmd = Twist()
        cmd.linear.x = acceleration_command[0]
        cmd.linear.y = acceleration_command[1]
        cmd.linear.z = acceleration_command[2]
        cmd.angular.x = torque_command[0]
        cmd.angular.y = torque_command[1]
        cmd.angular.z = torque_command[2]

        # Apply velocity limits
        # Linear velocity limits
        linear_velocity_xy_mag = np.linalg.norm([cmd.linear.x, cmd.linear.y])
        if linear_velocity_xy_mag > self.max_linear_velocity_xy and linear_velocity_xy_mag > 0:
            scale_factor = self.max_linear_velocity_xy / linear_velocity_xy_mag
            cmd.linear.x *= scale_factor
            cmd.linear.y *= scale_factor

        cmd.linear.z = np.clip(cmd.linear.z, -self.max_linear_velocity_z, self.max_linear_velocity_z)

        # Angular velocity limits
        cmd.angular.x = np.clip(cmd.angular.x, -self.max_angular_velocity_xy, self.max_angular_velocity_xy)
        cmd.angular.y = np.clip(cmd.angular.y, -self.max_angular_velocity_xy, self.max_angular_velocity_xy)
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_angular_velocity_z, self.max_angular_velocity_z)

        self.cmd_vel_pub.publish(cmd)

        # Debug logging (reduce frequency to avoid spam)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0

        if self._debug_counter % 25 == 0:  # Log every 25 iterations (0.5 seconds at 50Hz)
            self.get_logger().debug(
                f'Velocity control: vel=[{cmd.linear.x:.2f}, {cmd.linear.y:.2f}, {cmd.linear.z:.2f}], '
                f'ang_vel=[{cmd.angular.x:.2f}, {cmd.angular.y:.2f}, {cmd.angular.z:.2f}]'
            )

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()

    try:
        rclpy.spin(velocity_controller)
    except KeyboardInterrupt:
        pass

    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
