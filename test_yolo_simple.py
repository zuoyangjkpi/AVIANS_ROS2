#!/usr/bin/env python3
"""
Simple YOLO Test Script
======================

Test YOLO detector with proper model paths and error handling.
"""

import os
import sys
import subprocess
import time

def find_yolo_model():
    """Find YOLO model file in the workspace"""
    possible_paths = [
        "src/neural_network_detector/third_party/YOLOs-CPP/models/yolo12n.onnx",
        "src/neural_network_detector/third_party/YOLOs-CPP/models/yolo11n.onnx", 
        "src/neural_network_detector/third_party/YOLOs-CPP/models/yolo8n.onnx",
        "install/neural_network_detector/share/neural_network_detector/models/yolo12n.onnx",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return os.path.abspath(path)
    
    return None

def find_coco_labels():
    """Find COCO labels file"""
    possible_paths = [
        "src/neural_network_detector/third_party/YOLOs-CPP/models/coco.names",
        "install/neural_network_detector/share/neural_network_detector/models/coco.names",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return os.path.abspath(path)
    
    return None

def check_camera_topics():
    """Check if camera topics are available"""
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            camera_topics = [t for t in topics if 'camera' in t]
            return camera_topics
    except:
        pass
    return []

def run_yolo_detector(model_path, labels_path):
    """Run YOLO detector with specified paths"""
    
    print(f"🎯 Starting YOLO detector...")
    print(f"   Model: {model_path}")
    print(f"   Labels: {labels_path}")
    print(f"   Press Ctrl+C to stop")
    print()
    
    # Set parameters
    env = os.environ.copy()
    
    cmd = [
        'ros2', 'run', 'neural_network_detector', 'yolo12_detector_node',
        '--ros-args',
        '-p', f'model_path:={model_path}',
        '-p', f'labels_path:={labels_path}',
        '-p', 'use_gpu:=true',
        '-p', 'confidence_threshold:=0.5',
        '-p', 'desired_class:=0',  # Person class
        '-p', 'publish_debug_image:=true',
        '-p', 'max_update_rate_hz:=2.0'
    ]
    
    try:
        process = subprocess.Popen(cmd, env=env)
        process.wait()
    except KeyboardInterrupt:
        print("\n🛑 Stopping YOLO detector...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

def main():
    print("🔍 YOLO Detector Test")
    print("====================")
    
    # Check ROS2 environment
    if not os.environ.get('ROS_DISTRO'):
        print("❌ ROS2 environment not sourced")
        print("   Please run: source /opt/ros/jazzy/setup.bash")
        print("   And: source install/setup.bash")
        return
    
    print(f"✅ ROS2 {os.environ.get('ROS_DISTRO')} detected")
    
    # Find model files
    model_path = find_yolo_model()
    labels_path = find_coco_labels()
    
    print(f"\n📁 Checking model files...")
    if model_path:
        print(f"✅ Model found: {model_path}")
        # Check if it's a real model or dummy
        if os.path.getsize(model_path) < 1000:  # Less than 1KB is probably dummy
            print("⚠️  Warning: Model file seems to be a dummy/placeholder")
    else:
        print("❌ YOLO model not found!")
        print("   Run: ./fix_and_download_models.sh")
        return
    
    if labels_path:
        print(f"✅ Labels found: {labels_path}")
    else:
        print("❌ COCO labels not found!")
        print("   Run: ./fix_and_download_models.sh")
        return
    
    # Check camera topics
    print(f"\n📡 Checking camera topics...")
    camera_topics = check_camera_topics()
    if camera_topics:
        print(f"✅ Camera topics found:")
        for topic in camera_topics:
            print(f"   {topic}")
    else:
        print("⚠️  No camera topics found")
        print("   Start Gazebo first: ros2 launch drone_description gz.launch.py")
        print("   Or continue anyway to test YOLO initialization")
    
    # Ask user what to do
    print(f"\n🎯 Ready to test YOLO detector")
    
    if not camera_topics:
        print("⚠️  Warning: No camera topics detected")
        choice = input("Continue anyway? (y/n): ")
        if choice.lower() != 'y':
            print("👋 Exiting. Start Gazebo first, then run this script again.")
            return
    
    # Run YOLO detector
    try:
        run_yolo_detector(model_path, labels_path)
    except Exception as e:
        print(f"❌ Error running YOLO detector: {e}")
    
    print("\n✅ Test completed")

if __name__ == "__main__":
    main()