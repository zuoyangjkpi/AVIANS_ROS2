#!/usr/bin/env python3
"""
Simple Test Script for AVIANS ROS2 PORT1
========================================

This script provides simple commands to test the system step by step.
"""

import os
import sys
import subprocess

def run_command(cmd, description):
    """Run a command and show the result"""
    print(f"\n{'='*50}")
    print(f"Testing: {description}")
    print(f"Command: {cmd}")
    print('='*50)
    
    try:
        result = subprocess.run(cmd, shell=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ SUCCESS: {description}")
        else:
            print(f"❌ FAILED: {description} (exit code: {result.returncode})")
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"⏰ TIMEOUT: {description}")
        return False
    except Exception as e:
        print(f"💥 ERROR: {description} - {e}")
        return False

def main():
    print("AVIANS ROS2 PORT1 - Simple Test Script")
    print("=" * 50)
    
    # Environment check
    print(f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'Not set')}")
    print(f"Current directory: {os.getcwd()}")
    
    # Basic tests
    tests = [
        ("printenv ROS_DISTRO", "ROS2 installation"),
        ("ros2 pkg list | grep drone", "Drone packages"),
        ("ros2 pkg list | grep neural", "Neural network packages"),
        ("ros2 pkg prefix drone_description", "Drone description package"),
        ("ros2 pkg prefix neural_network_detector", "Neural network detector package"),
        ("ls install/", "Workspace installation"),
    ]
    
    passed = 0
    total = len(tests)
    
    for cmd, desc in tests:
        if run_command(cmd, desc):
            passed += 1
    
    print(f"\n{'='*50}")
    print(f"TEST SUMMARY: {passed}/{total} tests passed")
    print('='*50)
    
    if passed == total:
        print("\n🎉 All tests passed! You can try launching the simulation:")
        print("   ros2 launch drone_description gz.launch.py")
        print("   ros2 run neural_network_detector yolo12_detector_node")
    else:
        print(f"\n⚠️  {total-passed} tests failed. Please check your setup.")
        
    # Show available launch files
    print("\n📋 Available launch files:")
    run_command("find . -name '*.launch.py' | head -10", "Launch files")
    
    print("\n🔧 Manual testing commands:")
    print("   ros2 topic list")
    print("   ros2 node list") 
    print("   ros2 launch drone_description gz.launch.py")
    print("   ros2 run neural_network_detector yolo12_detector_node")

if __name__ == "__main__":
    main()