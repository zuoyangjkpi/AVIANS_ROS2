#!/bin/bash

# Gazebo Status Checker for ROS2 Jazzy
# ====================================

echo "🔍 Gazebo Status Check"
echo "====================="

# Check for Gazebo processes
echo "1. Checking Gazebo processes..."
GAZEBO_PROCS=$(pgrep -f "gz sim\|gazebo" | wc -l)
if [ $GAZEBO_PROCS -gt 0 ]; then
    echo "✅ Gazebo processes running: $GAZEBO_PROCS"
    pgrep -f "gz sim\|gazebo" | while read pid; do
        echo "   PID $pid: $(ps -p $pid -o comm=)"
    done
else
    echo "❌ No Gazebo processes found"
    echo ""
    echo "💡 To start Gazebo:"
    echo "   ros2 launch drone_description gz.launch.py"
fi

echo ""
echo "2. Checking ROS2 topics..."
timeout 5 ros2 topic list > /tmp/topics.txt 2>/dev/null
if [ $? -eq 0 ]; then
    TOTAL_TOPICS=$(wc -l < /tmp/topics.txt)
    CAMERA_TOPICS=$(grep -i camera /tmp/topics.txt | wc -l)
    
    echo "✅ ROS2 topics accessible"
    echo "   Total topics: $TOTAL_TOPICS"
    echo "   Camera topics: $CAMERA_TOPICS"
    
    if [ $CAMERA_TOPICS -gt 0 ]; then
        echo "   Camera topics found:"
        grep -i camera /tmp/topics.txt | sed 's/^/     /'
    fi
else
    echo "❌ Cannot access ROS2 topics"
    echo "   Check if ROS2 environment is sourced"
fi

echo ""
echo "3. Checking camera data..."
if grep -q "/camera/image_raw" /tmp/topics.txt 2>/dev/null; then
    echo "📷 Testing camera data..."
    timeout 3 ros2 topic hz /camera/image_raw 2>/dev/null | tail -1
    if [ $? -eq 0 ]; then
        echo "✅ Camera publishing data"
    else
        echo "⚠️  Camera topic exists but no data detected"
    fi
else
    echo "❌ No /camera/image_raw topic found"
fi

rm -f /tmp/topics.txt

echo ""
echo "📋 Summary:"
if [ $GAZEBO_PROCS -gt 0 ] && [ $CAMERA_TOPICS -gt 0 ]; then
    echo "✅ System ready for YOLO testing"
else
    echo "⚠️  System needs setup before YOLO testing"
fi