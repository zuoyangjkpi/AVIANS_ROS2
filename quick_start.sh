#!/bin/bash

# AVIANS ROS2 PORT1 - Quick Start Script (Fixed)
# ==============================================

echo "🚀 AVIANS ROS2 PORT1 - Quick Start (Fixed)"
echo "=========================================="

# Check if we're in the right directory
if [ ! -d "install" ]; then
    echo "❌ Error: install/ directory not found"
    echo "Please run this script from the workspace root"
    exit 1
fi

echo "✅ Workspace found"
echo "🎯 ROS_DISTRO: $ROS_DISTRO"

# Source the workspace (should already be done)
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced"
fi

echo ""
echo "Choose what to launch:"
echo "1) Full Gazebo simulation (recommended)"
echo "2) Simple Gazebo simulation"  
echo "3) YOLO detector only"
echo "4) Show available launch files"
echo "5) Monitor topics"
echo "6) Kill all ROS processes"
echo "0) Exit"

read -p "Enter choice (0-6): " choice

case $choice in
    1)
        echo "🚀 Launching full simulation..."
        echo "📝 This will start:"
        echo "   - Gazebo world with drone"
        echo "   - YOLO12 person detection"
        echo "   - 3D projection model"
        echo "   - Kalman filter tracking"
        echo "   - RViz visualization"
        echo ""
        echo "Press Ctrl+C to stop the simulation"
        echo "Use './kill.sh' in another terminal if needed"
        echo ""
        ros2 launch drone_state_publisher simulation.launch.py
        ;;
    2)
        echo "🚀 Launching simple Gazebo..."
        echo "📝 This will start basic Gazebo simulation"
        echo ""
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 launch drone_description gz.launch.py
        ;;
    3)
        echo "🚀 Launching YOLO detector..."
        echo "📝 Make sure camera topics are available"
        echo "   Run 'ros2 topic list' to check"
        echo ""
        ros2 run neural_network_detector yolo12_detector_node
        ;;
    4)
        echo "📋 Available launch files:"
        echo ""
        echo "🎯 Main launch files:"
        echo "   ros2 launch drone_state_publisher simulation.launch.py  # Full system"
        echo "   ros2 launch drone_description gz.launch.py              # Simple Gazebo"
        echo ""
        echo "🔧 Component launch files:"
        find install/ -name "*.launch.py" | sort | while read file; do
            package=$(echo $file | cut -d'/' -f2)
            launch_file=$(basename $file)
            echo "   ros2 launch $package $launch_file"
        done
        echo ""
        ;;
    5)
        echo "📊 Monitoring ROS2 topics..."
        echo ""
        echo "🔍 Available topics:"
        ros2 topic list 2>/dev/null || echo "❌ No ROS2 daemon running"
        echo ""
        echo "📈 Key topics to monitor:"
        echo "   ros2 topic echo /camera/image_raw       # Camera feed"
        echo "   ros2 topic echo /person_detections      # YOLO detections"  
        echo "   ros2 topic echo /X3/odom                # Drone odometry"
        echo "   ros2 topic hz /camera/image_raw         # Camera frequency"
        echo ""
        echo "🎮 Control topics:"
        echo "   ros2 topic pub /X3/cmd_vel geometry_msgs/msg/Twist ..."
        echo ""
        ;;
    6)
        echo "💀 Killing all ROS processes..."
        if [ -f "./kill.sh" ]; then
            chmod +x ./kill.sh
            ./kill.sh
        else
            echo "❌ kill.sh not found in current directory"
            echo "Trying manual cleanup..."
            pkill -f "ros2|gazebo|gz|rviz" 2>/dev/null || true
            echo "✅ Basic cleanup done"
        fi
        ;;
    0)
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice"
        echo ""
        echo "💡 Quick commands:"
        echo "   ./quick_start_fixed.sh    # Run this menu again"
        echo "   ./kill.sh                 # Kill all ROS processes"
        echo "   ros2 topic list           # Show active topics"
        echo "   ros2 node list            # Show active nodes"
        exit 1
        ;;
esac