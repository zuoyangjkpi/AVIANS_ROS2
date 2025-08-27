#!/usr/bin/env python3
"""
Minimal YOLO Test - Step by Step Debugging
==========================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading

class MinimalYOLOTest(Node):
    def __init__(self):
        super().__init__('minimal_yolo_test')
        
        print("🔍 Step 1: Node created")
        
        self.bridge = CvBridge()
        self.image_count = 0
        self.last_log_time = time.time()
        
        print("🔍 Step 2: Bridge initialized")
        
        # Create subscription with different QoS settings
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        print("🔍 Step 3: QoS profile created")
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        print("🔍 Step 4: Subscription created")
        print("🔍 Step 5: Waiting for images...")
        print("   - Topic: /camera/image_raw")
        print("   - QoS: Best effort, depth=1")
        print("   - Press Ctrl+C to stop")
        
        # Create a timer to show we're alive
        self.timer = self.create_timer(2.0, self.heartbeat)
        
    def heartbeat(self):
        """Show that the node is still alive"""
        current_time = time.time()
        print(f"💓 Heartbeat: {current_time:.1f}s, Images received: {self.image_count}")
        
    def image_callback(self, msg):
        """Process incoming images with detailed logging"""
        callback_start = time.time()
        
        try:
            self.image_count += 1
            
            print(f"\n📷 Image {self.image_count} received:")
            print(f"   Size: {msg.width}x{msg.height}")
            print(f"   Encoding: {msg.encoding}")
            print(f"   Data length: {len(msg.data)} bytes")
            print(f"   Frame ID: {msg.header.frame_id}")
            print(f"   Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            
            # Try to convert image
            print("   🔄 Converting image...")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print(f"   ✅ Converted to OpenCV: {cv_image.shape}")
            
            # Simulate lightweight processing
            print("   🧠 Simulating processing...")
            time.sleep(0.01)  # 10ms processing time
            
            callback_time = (time.time() - callback_start) * 1000
            print(f"   ⏱️  Callback time: {callback_time:.1f}ms")
            
            # Log every 10 images
            if self.image_count % 10 == 0:
                print(f"\n📊 Summary: {self.image_count} images processed successfully")
                
        except Exception as e:
            print(f"   ❌ Error in callback: {e}")
            import traceback
            traceback.print_exc()

def main():
    print("🧪 Minimal YOLO Test")
    print("====================")
    print("This will test image reception step by step")
    print()
    
    rclpy.init()
    
    try:
        node = MinimalYOLOTest()
        print("\n✅ Node initialized successfully")
        print("🔄 Starting spin...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("\n🏁 Test completed")

if __name__ == '__main__':
    main()
