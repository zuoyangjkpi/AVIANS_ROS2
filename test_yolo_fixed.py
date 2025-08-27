#!/usr/bin/env python3
"""
YOLO Test with Fixed Paths
"""

import subprocess
import os
import sys

def main():
    print("🎯 YOLO Detector Test (Fixed Paths)")
    print("==================================")
    
    # Use absolute paths
    model_path = "/home/zuoyangjkpi/AVIANS_ROS2_PORT1/src/neural_network_detector/third_party/YOLOs-CPP/models/yolo12n.onnx"
    labels_path = "/home/zuoyangjkpi/AVIANS_ROS2_PORT1/src/neural_network_detector/third_party/YOLOs-CPP/quantized_models/coco.names"
    
    print(f"Model:  {model_path}")
    print(f"Labels: {labels_path}")
    print()
    
    # Check files exist
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        return
    if not os.path.exists(labels_path):
        print(f"❌ Labels not found: {labels_path}")
        return
    
    print("✅ Files verified, starting YOLO detector...")
    print("Press Ctrl+C to stop")
    print()
    
    cmd = [
        'ros2', 'run', 'neural_network_detector', 'yolo12_detector_node',
        '--ros-args',
        '-p', f'model_path:={model_path}',
        '-p', f'labels_path:={labels_path}',
        '-p', 'use_gpu:=true',
        '-p', 'confidence_threshold:=0.3',
        '-p', 'desired_class:=0',  # Person class
        '-p', 'publish_debug_image:=true',
        '-p', 'max_update_rate_hz:=2.0'
    ]
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n🛑 Stopped")

if __name__ == "__main__":
    main()
