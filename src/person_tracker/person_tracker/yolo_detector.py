#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from tf_transformations import quaternion_from_euler
import math

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Initialize YOLO model
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO('yolo12n.pt')
        self.get_logger().info("YOLO model loaded successfully!")
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Detection parameters
        self.conf_threshold = 0.5
        self.frame_skip = 2
        self.frame_count = 0
        
        # Camera parameters (adjust based on your drone camera)
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov_h = 1.047  # ~60 degrees horizontal FOV
        
        # Tracking variables
        self.target_person = None
        self.no_detection_count = 0
        self.max_no_detection = 10  # frames
        
        # ROS2 Publishers and Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            # '/firefly_1/xtion/rgb/Image_raw',  # Adjust topic name to match your Gazebo camera
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/target_waypoint',  
            10
        )
        
        # Publisher for debug images
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/yolo_debug_image',
            10
        )
        
        # Parameters
        self.declare_parameter('target_distance', 3.0)  # meters
        self.declare_parameter('target_height', 3.0)    # meters above person
        
        self.get_logger().info("YOLO Detector Node initialized!")
        self.get_logger().info(f"Subscribed to: {self.image_subscriber.topic_name}")
        self.get_logger().info(f"Publishing poses to: {self.pose_publisher.topic_name}")

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Skip frames for performance
            self.frame_count += 1
            if self.frame_count % self.frame_skip != 0:
                return
            
            # Detect persons
            persons = self.detect_persons(cv_image)
            
            # Track and select target
            target = self.select_target_person(persons, cv_image)
            
            if target:
                # Convert detection to target pose
                target_pose = self.detection_to_pose(target, msg.header)
                self.pose_publisher.publish(target_pose)
                
                self.no_detection_count = 0
                self.get_logger().info(f"Target detected at center: ({target['center'][0]:.1f}, {target['center'][1]:.1f})")
            else:
                self.no_detection_count += 1
                if self.no_detection_count > self.max_no_detection:
                    self.get_logger().warn("No person detected - target lost!")
            
            # Publish debug image
            debug_image = self.draw_debug_info(cv_image, persons, target)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_image_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def detect_persons(self, image):
        """Detect persons using YOLO"""
        results = self.model(image, classes=[0], conf=self.conf_threshold, verbose=False)
        
        persons = []
        if results[0].boxes is not None:
            for box in results[0].boxes:
                bbox = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                confidence = float(box.conf[0])
                
                # Calculate center and area
                center_x = (bbox[0] + bbox[2]) / 2
                center_y = (bbox[1] + bbox[3]) / 2
                area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                
                persons.append({
                    'bbox': bbox,
                    'center': [center_x, center_y],
                    'confidence': confidence,
                    'area': area
                })
        
        return persons

    def select_target_person(self, persons, image):
        """Select which person to track (largest/closest person)"""
        if not persons:
            return None
        
        # For now, select the person with largest area (closest/most prominent)
        target = max(persons, key=lambda p: p['area'])
        
        # Store for consistency
        self.target_person = target
        return target

    def detection_to_pose(self, detection, header):
        """Convert 2D detection to 3D target pose for drone"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "world"  
        
        # Get detection center in image coordinates
        center_x, center_y = detection['center']
        
        # Convert to normalized coordinates (-1 to 1)
        norm_x = (center_x - self.camera_width/2) / (self.camera_width/2)
        norm_y = (center_y - self.camera_height/2) / (self.camera_height/2)
        
        # Calculate target position relative to drone
        target_distance = self.get_parameter('target_distance').value
        target_height = self.get_parameter('target_height').value
        
        # Calculate angle offset based on normalized position
        angle_x = norm_x * (self.camera_fov_h / 2)  # Horizontal angle offset
        angle_y = norm_y * (self.camera_fov_h / 2)  # Vertical angle offset (assuming same FOV)
        
        # Calculate target position in 3D space
        # This assumes the drone is hovering and looking down/forward at the person
        target_x = target_distance * math.sin(angle_x)  # Left/Right offset
        target_y = target_distance * math.cos(angle_y)  # Forward distance to person
        target_z = target_height  # Height above person
        
        # Set position
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = target_z
        
        # Calculate yaw to face the person
        target_yaw = angle_x  # Yaw towards the detected person
        
        # Convert yaw to quaternion
        quaternion = quaternion_from_euler(0, 0, target_yaw)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        return pose_msg

    def draw_debug_info(self, image, persons, target):
        """Draw detection results on image for debugging"""
        debug_image = image.copy()
        
        # Draw all detections
        for person in persons:
            bbox = person['bbox']
            conf = person['confidence']
            
            # Color: green for target, blue for others
            color = (255, 0, 0) if person == target else (0, 0, 255)
            
            # Draw bounding box
            cv2.rectangle(debug_image, 
                         (int(bbox[0]), int(bbox[1])), 
                         (int(bbox[2]), int(bbox[3])), 
                         color, 2)
            
            # Draw center point
            center = person['center']
            cv2.circle(debug_image, (int(center[0]), int(center[1])), 5, color, -1)
            
            # Add confidence text
            cv2.putText(debug_image, f'{conf:.2f}', 
                       (int(bbox[0]), int(bbox[1])-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw crosshairs for image center
        h, w = debug_image.shape[:2]
        cv2.line(debug_image, (w//2-20, h//2), (w//2+20, h//2), (255, 255, 255), 2)
        cv2.line(debug_image, (w//2, h//2-20), (w//2, h//2+20), (255, 255, 255), 2)
        
        # Add status text
        status = f"Persons: {len(persons)}"
        if target:
            status += f" | Target: LOCKED"
        else:
            status += f" | Target: LOST ({self.no_detection_count})"
            
        cv2.putText(debug_image, status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return debug_image

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()