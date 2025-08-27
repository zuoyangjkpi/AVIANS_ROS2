#!/bin/bash

echo "🎯 Simple YOLO Launcher"
echo "======================"

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ Environment sourced"
echo ""

# Check if camera is available
echo "🔍 Checking for camera topics..."
timeout 3 ros2 topic list | grep camera > /dev/null
if [ $? -eq 0 ]; then
    echo "✅ Camera topics found"
    ros2 topic list | grep camera
else
    echo "⚠️  No camera topics found"
    echo "   Start Gazebo first: ros2 launch drone_description gz.launch.py"
    echo "   Or continue anyway to test YOLO initialization"
fi

echo ""
echo "🚀 Starting YOLO detector with fixed paths..."
echo "Press Ctrl+C to stop"
echo ""

./test_yolo_fixed.py
