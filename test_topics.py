#!/usr/bin/env python3
"""
Topic Monitor for AVIANS ROS2 PORT1
===================================

This script monitors key ROS2 topics to verify the simulation is working correctly.

Usage:
    chmod +x test_topics.py
    ./test_topics.py
"""

import subprocess
import time
import sys

def run_command(cmd, timeout=5):
    """Run a command with timeout"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, 
                              text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Timeout"
    except Exception as e:
        return False, "", str(e)

def check_topic_list():
    """Check if key topics exist"""
    print("Checking ROS2 topics...")
    
    success, stdout, stderr = run_command("ros2 topic list", timeout=10)
    if not success:
        print(f"[ERROR] Failed to get topic list: {stderr}")
        return False
        
    topics = stdout.strip().split('\n')
    
    expected_topics = [
        '/camera/image_raw',
        '/camera/camera_info', 
        '/person_detections',
        '/detection_debug_image',
        '/machine_1/pose',
        '/machine_1/target_tracker/pose',
        '/X3/cmd_vel',
        '/X3/odom'
    ]
    
    print(f"\nFound {len(topics)} topics:")
    for topic in sorted(topics):
        if topic.strip():
            status = "✓" if topic in expected_topics else " "
            print(f"  {status} {topic}")
    
    missing = [t for t in expected_topics if t not in topics]
    if missing:
        print(f"\nMissing expected topics:")
        for topic in missing:
            print(f"  ✗ {topic}")
    else:
        print(f"\n✓ All expected topics found!")
        
    return len(missing) == 0

def check_topic_hz(topic, expected_hz=None):
    """Check topic publishing rate"""
    print(f"\nChecking {topic} publishing rate...")
    
    success, stdout, stderr = run_command(f"timeout 5 ros2 topic hz {topic}", timeout=10)
    
    if success and "average rate:" in stdout:
        lines = stdout.strip().split('\n')
        for line in lines:
            if "average rate:" in line:
                print(f"  ✓ {line}")
                return True
    else:
        print(f"  ✗ No data or error: {stderr}")
        return False
        
def check_topic_echo(topic, lines=3):
    """Check topic content"""
    print(f"\nChecking {topic} content...")
    
    success, stdout, stderr = run_command(f"timeout 3 ros2 topic echo {topic} --once", timeout=5)
    
    if success and stdout.strip():
        print(f"  ✓ Topic publishing data")
        # Show first few lines
        content_lines = stdout.strip().split('\n')[:lines]
        for line in content_lines:
            print(f"    {line}")
        if len(stdout.strip().split('\n')) > lines:
            print("    ...")
        return True
    else:
        print(f"  ✗ No data: {stderr}")
        return False

def main():
    print("="*60)
    print("AVIANS ROS2 PORT1 - Topic Monitor")
    print("="*60)
    
    # Check if ROS2 is available
    success, _, _ = run_command("printenv ROS_DISTRO")
    if not success:
        print("[ERROR] ROS2 not available")
        sys.exit(1)
        
    # Check topic list
    if not check_topic_list():
        print("\n[WARNING] Some expected topics are missing")
        
    # Check key topics
    key_topics = [
        '/camera/image_raw',
        '/person_detections', 
        '/machine_1/pose',
        '/X3/odom'
    ]
    
    print("\n" + "="*40)
    print("CHECKING KEY TOPICS")
    print("="*40)
    
    for topic in key_topics:
        check_topic_hz(topic)
        check_topic_echo(topic)
        print("-" * 40)
        
    print("\n[INFO] Topic monitoring complete")
    print("Use 'ros2 topic list' to see all topics")
    print("Use 'ros2 topic echo <topic>' to monitor specific topics")

if __name__ == "__main__":
    main()
