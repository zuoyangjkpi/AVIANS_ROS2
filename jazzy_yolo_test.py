#!/usr/bin/env python3
"""
YOLO Test for ROS2 Jazzy + Python 3.12
======================================
"""

import os
import sys
import subprocess
import time

def check_environment():
    """Check ROS2 Jazzy environment"""
    print("🔍 Environment Check")
    print("=" * 20)
    
    # Check ROS2 Jazzy
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro == 'jazzy':
        print(f"✅ ROS2 Jazzy detected")
    else:
        print(f"⚠️  ROS_DISTRO: {ros_distro} (expected: jazzy)")
    
    # Check Python version
    python_version = sys.version
    print(f"🐍 Python: {python_version.split()[0]}")
    
    # Check if conda environment is active
    conda_env = os.environ.get('CONDA_DEFAULT_ENV')
    if conda_env:
        print(f"🐍 Conda environment: {conda_env}")
    
    return True

def check_camera_topics():
    """Check camera topics in Jazzy"""
    print("\n📷 Camera Topics Check")
    print("=" * 25)
    
    try:
        # Check topic list
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            camera_topics = [t for t in topics if 'camera' in t.lower()]
            
            print(f"📡 Total topics: {len(topics)}")
            
            if camera_topics:
                print("✅ Camera topics found:")
                for topic in camera_topics:
                    print(f"   {topic}")
                    
                # Check camera topic frequency
                print("\n📊 Checking camera frequency...")
                freq_result = subprocess.run(['timeout', '3', 'ros2', 'topic', 'hz', '/camera/image_raw'], 
                                           capture_output=True, text=True)
                if freq_result.returncode == 0 and 'average rate' in freq_result.stdout:
                    print("✅ Camera publishing data")
                else:
                    print("⚠️  Camera topic exists but no data")
                    
            else:
                print("❌ No camera topics found")
                print("   Available topics:")
                for topic in topics[:10]:
                    print(f"   {topic}")
                    
            return len(camera_topics) > 0
            
        else:
            print(f"❌ Failed to get topic list: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error checking topics: {e}")
        return False

def find_yolo_model():
    """Find YOLO model in Jazzy environment"""
    print("\n🎯 YOLO Model Check")
    print("=" * 20)
    
    possible_paths = [
        "src/neural_network_detector/third_party/models/yolo12n.onnx",
        "src/neural_network_detector/third_party/YOLOs-CPP/models/yolo12n.onnx",
        "install/neural_network_detector/share/neural_network_detector/models/yolo12n.onnx",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            size = os.path.getsize(path) / 1024 / 1024
            print(f"✅ Model found: {path} ({size:.1f} MB)")
            return os.path.abspath(path)
    
    print("❌ No YOLO model found")
    return None

def find_coco_labels():
    """Find COCO labels"""
    possible_paths = [
        "src/neural_network_detector/third_party/models/coco.names",
        "src/neural_network_detector/third_party/YOLOs-CPP/models/coco.names",
        "src/neural_network_detector/third_party/YOLOs-CPP/quantized_models/coco.names",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            print(f"✅ Labels found: {path}")
            return os.path.abspath(path)
    
    print("❌ No COCO labels found")
    return None

def test_yolo_imports():
    """Test YOLO-related imports"""
    print("\n📦 Import Test")
    print("=" * 15)
    
    imports_status = {}
    
    # Test ROS2 imports
    try:
        import rclpy
        print("✅ rclpy")
        imports_status['rclpy'] = True
    except ImportError as e:
        print(f"❌ rclpy: {e}")
        imports_status['rclpy'] = False
    
    # Test sensor messages
    try:
        from sensor_msgs.msg import Image
        print("✅ sensor_msgs")
        imports_status['sensor_msgs'] = True
    except ImportError as e:
        print(f"❌ sensor_msgs: {e}")
        imports_status['sensor_msgs'] = False
    
    # Test OpenCV
    try:
        import cv2
        print(f"✅ OpenCV {cv2.__version__}")
        imports_status['opencv'] = True
    except ImportError as e:
        print(f"❌ OpenCV: {e}")
        imports_status['opencv'] = False
    
    # Test ONNX Runtime
    try:
        import onnxruntime as ort
        providers = ort.get_available_providers()
        print(f"✅ ONNX Runtime (providers: {len(providers)})")
        imports_status['onnxruntime'] = True
    except ImportError as e:
        print(f"❌ ONNX Runtime: {e}")
        imports_status['onnxruntime'] = False
    
    return imports_status

def run_yolo_detector(model_path, labels_path):
    """Run YOLO detector with proper parameters"""
    print("\n🚀 Starting YOLO Detector")
    print("=" * 30)
    
    print(f"Model:  {model_path}")
    print(f"Labels: {labels_path}")
    print()
    print("🎯 YOLO Detector Parameters:")
    print("   - use_gpu: true (GPU mode)")
    print("   - confidence_threshold: 0.3")
    print("   - desired_class: 0 (person)")
    print("   - max_update_rate_hz: 2.0")
    print()
    print("📡 Expected topics:")
    print("   - Input:  /camera/image_raw")
    print("   - Output: /person_detections")
    print()
    print("Press Ctrl+C to stop")
    print("-" * 30)
    
    cmd = [
        'ros2', 'run', 'neural_network_detector', 'yolo12_detector_node',
        '--ros-args',
        '-p', f'model_path:={model_path}',
        '-p', f'labels_path:={labels_path}',
        '-p', 'use_gpu:=true',
        '-p', 'confidence_threshold:=0.3',
        '-p', 'desired_class:=0',
        '-p', 'publish_debug_image:=true',
        '-p', 'max_update_rate_hz:=2.0',
        '--log-level', 'INFO'
    ]
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n🛑 YOLO detector stopped")

def main():
    print("🎯 YOLO Test for ROS2 Jazzy + Python 3.12")
    print("=" * 50)
    
    # Check environment
    check_environment()
    
    # Check camera topics
    camera_ok = check_camera_topics()
    
    # Find model files
    model_path = find_yolo_model()
    labels_path = find_coco_labels()
    
    # Test imports
    imports = test_yolo_imports()
    
    # Summary
    print("\n📋 Diagnostic Summary")
    print("=" * 25)
    print(f"Camera topics:    {'✅' if camera_ok else '❌'}")
    print(f"YOLO model:       {'✅' if model_path else '❌'}")
    print(f"COCO labels:      {'✅' if labels_path else '❌'}")
    print(f"ROS2 imports:     {'✅' if imports.get('rclpy') else '❌'}")
    print(f"ONNX Runtime:     {'✅' if imports.get('onnxruntime') else '❌'}")
    
    # Recommendations
    print("\n💡 Recommendations:")
    if not camera_ok:
        print("   1. Start Gazebo: ros2 launch drone_description gz.launch.py")
    if not model_path:
        print("   2. Check YOLO model location")
    if not imports.get('onnxruntime'):
        print("   3. Install ONNX Runtime: pip install onnxruntime")
    if not imports.get('rclpy'):
        print("   4. Check ROS2 Jazzy installation")
    
    # Run YOLO if everything looks good
    if camera_ok and model_path and labels_path and imports.get('rclpy'):
        print("\n🚀 All checks passed! Starting YOLO detector...")
        input("Press Enter to continue or Ctrl+C to exit...")
        run_yolo_detector(model_path, labels_path)
    else:
        print("\n⚠️  Please fix the issues above before running YOLO detector")

if __name__ == "__main__":
    main()