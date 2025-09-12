#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
import time

class PoseToOdomBridge(Node):
    def __init__(self):
        super().__init__('pose_to_odom_bridge')
        
        # Subscribe to Gazebo pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/world/multicopter/pose/info',  # This is the default Gazebo pose topic
            self.pose_callback,
            10
        )
        
        # Alternative pose subscribers for different possible topics
        self.pose_sub2 = self.create_subscription(
            PoseStamped,
            '/model/X3/pose',
            self.pose_callback,
            10
        )
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/X3/odometry', 10)
        
        self.get_logger().info('Pose to Odometry bridge started')
        
        # Store previous pose for velocity calculation
        self.prev_pose = None
        self.prev_time = time.time()
        
    def pose_callback(self, msg):
        # Create odometry message
        odom = Odometry()
        
        # Header
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'X3/odom'
        odom.child_frame_id = 'X3/base_link'
        
        # Position and orientation from pose
        odom.pose.pose = msg.pose
        
        # Calculate velocity if we have previous pose
        current_time = time.time()
        if self.prev_pose is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                # Linear velocity
                dx = msg.pose.position.x - self.prev_pose.position.x
                dy = msg.pose.position.y - self.prev_pose.position.y
                dz = msg.pose.position.z - self.prev_pose.position.z
                
                odom.twist.twist.linear.x = dx / dt
                odom.twist.twist.linear.y = dy / dt
                odom.twist.twist.linear.z = dz / dt
        
        # Store current pose for next iteration
        self.prev_pose = msg.pose
        self.prev_time = current_time
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        self.get_logger().info(f'Published odometry: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()