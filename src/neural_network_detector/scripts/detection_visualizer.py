#!/usr/bin/python3
"""
Python-based Detection Visualizer for RViz
Creates detection visualization images with bounding boxes
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from neural_network_msgs.msg import NeuralNetworkDetectionArray

class DetectionVisualizerPy(Node):
    def __init__(self):
        super().__init__('detection_visualizer_py')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', 
            self.image_callback, 10
        )
        
        self.detection_sub = self.create_subscription(
            NeuralNetworkDetectionArray, '/person_detections',
            self.detection_callback, 10
        )
        
        # Publisher
        self.vis_pub = self.create_publisher(
            Image, '/detection_image', 10
        )
        
        self.get_logger().info("Python Detection Visualizer started")
    
    def image_callback(self, msg):
        self.latest_image = msg
        self.publish_visualization()
    
    def detection_callback(self, msg):
        self.latest_detections = msg
        self.publish_visualization()
    
    def publish_visualization(self):
        if self.latest_image is None or self.latest_detections is None:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            
            # Draw detections
            for detection in self.latest_detections.detections:
                if detection.object_class == 1:  # Person class
                    # Draw green bounding box
                    cv2.rectangle(cv_image, 
                                (int(detection.xmin), int(detection.ymin)),
                                (int(detection.xmax), int(detection.ymax)),
                                (0, 255, 0), 3)
                    
                    # Add confidence text
                    confidence = int(detection.detection_score * 100)
                    label = f"Person: {confidence}%"
                    cv2.putText(cv_image, label,
                              (int(detection.xmin), int(detection.ymin) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Convert back to ROS message
            vis_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            vis_msg.header = self.latest_image.header
            
            # Publish visualization
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f"Visualization error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizerPy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()