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
            base_transform = TransformStamped()
            base_transform.header.stamp = self.get_clock().now().to_msg()
            base_transform.header.frame_id = 'world'
            base_transform.child_frame_id = 'X3/base_link'

            # Copy position
            base_transform.transform.translation.x = msg.pose.pose.position.x
            base_transform.transform.translation.y = msg.pose.pose.position.y
            base_transform.transform.translation.z = msg.pose.pose.position.z

            # Copy orientation
            base_transform.transform.rotation = msg.pose.pose.orientation

            # Create static transform from X3/base_link to machine_1_camera_link
            # Camera is positioned 0.2m forward from base_link (based on URDF)
            camera_transform = TransformStamped()
            camera_transform.header.stamp = self.get_clock().now().to_msg()
            camera_transform.header.frame_id = 'X3/base_link'
            camera_transform.child_frame_id = 'machine_1_camera_link'

            # Camera position relative to base_link (from URDF)
            camera_transform.transform.translation.x = 0.2  # 0.2m forward
            camera_transform.transform.translation.y = 0.0
            camera_transform.transform.translation.z = 0.0

            # Camera orientation (no rotation relative to base_link)
            camera_transform.transform.rotation.x = 0.0
            camera_transform.transform.rotation.y = 0.0
            camera_transform.transform.rotation.z = 0.0
            camera_transform.transform.rotation.w = 1.0

            # Broadcast both transforms
            self.tf_broadcaster.sendTransform([base_transform, camera_transform])

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