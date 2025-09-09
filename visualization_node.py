#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from neural_network_msgs.msg import NeuralNetworkDetectionArray
from nav_msgs.msg import Odometry
import math

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Publishers
        self.person_marker_pub = self.create_publisher(MarkerArray, '/person_position_markers', 10)
        self.trajectory_marker_pub = self.create_publisher(MarkerArray, '/drone_trajectory_markers', 10)
        self.drone_path_pub = self.create_publisher(Path, '/drone_path', 10)
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            NeuralNetworkDetectionArray, '/person_detections', self.detection_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10)
        
        # Timer for publishing trajectory
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        
        # Data storage
        self.latest_person_detections = []
        self.drone_path = Path()
        self.drone_path.header.frame_id = "world"
        
        # Trajectory parameters (circular orbit around person)
        self.orbit_radius = 3.0  # meters
        self.orbit_height = 2.5  # meters above person
        self.current_person_position = [0.0, 0.0, 1.0]  # Default position
        
        self.get_logger().info('Visualization node started')

    def detection_callback(self, msg):
        """Process person detections and create position markers"""
        self.latest_person_detections = msg.detections
        
        # Create person position markers
        marker_array = MarkerArray()
        
        for i, detection in enumerate(msg.detections):
            # Create a marker for each person detection
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "person"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Estimate 3D position from 2D detection (simplified)
            # This is a rough estimation - in reality you'd need depth info
            center_x = (detection.xmin + detection.xmax) / 2.0
            center_y = (detection.ymin + detection.ymax) / 2.0
            
            # Convert image coordinates to world coordinates (rough approximation)
            # Assuming camera is at drone position looking down
            world_x = (center_x - 320) / 100.0  # Scale factor
            world_y = (center_y - 240) / 100.0  # Scale factor
            world_z = 1.0  # Assume person is at ground level + height
            
            # Update current person position
            self.current_person_position = [world_x, world_y, world_z]
            
            # Set marker position
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = world_z
            marker.pose.orientation.w = 1.0
            
            # Set marker properties
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.8  # Human height
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # Clear old markers if no detections
        if not msg.detections:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "person"
            marker.id = 0
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
        
        self.person_marker_pub.publish(marker_array)

    def odometry_callback(self, msg):
        """Update drone path with current position"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.drone_path.poses.append(pose)
        
        # Keep only last 100 poses to avoid memory issues
        if len(self.drone_path.poses) > 100:
            self.drone_path.poses.pop(0)
        
        self.drone_path.header.stamp = self.get_clock().now().to_msg()
        self.drone_path_pub.publish(self.drone_path)

    def publish_trajectory(self):
        """Publish desired circular trajectory around person"""
        marker_array = MarkerArray()
        
        # Create circular trajectory markers
        num_points = 32
        person_x, person_y, person_z = self.current_person_position
        
        # Create trajectory circle
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0
        
        # Set marker properties
        marker.scale.x = 0.1  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # Generate circular points
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2.0 * math.pi * i / num_points
            point = Point()
            point.x = person_x + self.orbit_radius * math.cos(angle)
            point.y = person_y + self.orbit_radius * math.sin(angle)
            point.z = person_z + self.orbit_height
            marker.points.append(point)
        
        marker_array.markers.append(marker)
        
        # Create center marker (person position indicator)
        center_marker = Marker()
        center_marker.header.frame_id = "world"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "trajectory"
        center_marker.id = 1
        center_marker.type = Marker.CYLINDER
        center_marker.action = Marker.ADD
        
        center_marker.pose.position.x = person_x
        center_marker.pose.position.y = person_y
        center_marker.pose.position.z = person_z + self.orbit_height / 2
        center_marker.pose.orientation.w = 1.0
        
        center_marker.scale.x = 0.2
        center_marker.scale.y = 0.2
        center_marker.scale.z = self.orbit_height
        center_marker.color.r = 1.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.color.a = 0.3
        
        marker_array.markers.append(center_marker)
        
        self.trajectory_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()