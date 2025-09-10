#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DroneTfPublisher(Node):
    def __init__(self):
        super().__init__('drone_tf_publisher')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to drone odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odom_callback, 10)
        
        self.get_logger().info('Drone TF Publisher started - bridging /X3/odometry to TF tree')
        
        # Timer to log TF publishing status
        self.status_timer = self.create_timer(2.0, self.log_status)

    def odom_callback(self, msg):
        """Convert odometry to TF transform"""
        try:
            # Create transform from world to drone base_link
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'X3/base_link'
            
            # Copy position
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y  
            transform.transform.translation.z = msg.pose.pose.position.z
            
            # Copy orientation
            transform.transform.rotation = msg.pose.pose.orientation
            
            # Broadcast transform
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish TF: {e}')
    
    def log_status(self):
        """Log current status for debugging"""
        try:
            self.get_logger().info('TF Publisher running - listening for /X3/odometry')
        except Exception as e:
            self.get_logger().error(f'Status error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneTfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()