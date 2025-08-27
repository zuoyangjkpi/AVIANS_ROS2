#!/bin/bash

# AVIANS ROS2 PORT1 - Kill All ROS Processes
# ==========================================

echo "🔥 AVIANS ROS2 PORT1 - Process Killer"
echo "====================================="

# Function to kill processes by name pattern
kill_processes() {
    local pattern=$1
    local description=$2
    
    echo "🔍 Looking for $description processes..."
    
    # Find processes matching the pattern
    pids=$(pgrep -f "$pattern" 2>/dev/null)
    
    if [ -n "$pids" ]; then
        echo "📋 Found processes: $pids"
        echo "💀 Killing $description processes..."
        
        # First try SIGTERM (graceful shutdown)
        kill $pids 2>/dev/null
        sleep 2
        
        # Check if any are still running and force kill
        remaining=$(pgrep -f "$pattern" 2>/dev/null)
        if [ -n "$remaining" ]; then
            echo "⚡ Force killing remaining processes: $remaining"
            kill -9 $remaining 2>/dev/null
        fi
        
        echo "✅ $description processes terminated"
    else
        echo "✅ No $description processes found"
    fi
    echo ""
}

# Kill ROS2 related processes
echo "Starting ROS2 process cleanup..."
echo ""

# Kill Gazebo processes
kill_processes "gz sim" "Gazebo Simulation"
kill_processes "gazebo" "Gazebo (legacy)"
kill_processes "gzserver" "Gazebo Server"
kill_processes "gzclient" "Gazebo Client"

# Kill RViz
kill_processes "rviz2" "RViz2"

# Kill ROS2 nodes
kill_processes "ros2" "ROS2 commands"
kill_processes "parameter_bridge" "ROS-Gazebo Bridge"

# Kill specific project nodes
kill_processes "yolo12_detector_node" "YOLO12 Detector"
kill_processes "neural_network_detector" "Neural Network Detector"
kill_processes "projection_model_node" "Projection Model"
kill_processes "distributed_kf_node" "Distributed Kalman Filter"
kill_processes "drone_state_publisher" "Drone State Publisher"
kill_processes "waypoint_controller" "Waypoint Controller"
kill_processes "tf_from_uav_pose" "TF Publisher"

# Kill Python launch processes
kill_processes "python.*launch" "Python Launch Files"

# Kill any remaining ROS processes
kill_processes "_ros" "ROS Internal Processes"

# Clean up shared memory and temporary files
echo "🧹 Cleaning up system resources..."

# Remove ROS2 log files (optional - comment out if you want to keep logs)
if [ -d "$HOME/.ros/log" ]; then
    echo "🗑️  Cleaning old ROS2 logs..."
    find "$HOME/.ros/log" -type f -mtime +1 -delete 2>/dev/null || true
fi

# Clean up Gazebo temporary files
if [ -d "/tmp/gazebo" ]; then
    echo "🗑️  Cleaning Gazebo temp files..."
    rm -rf /tmp/gazebo* 2>/dev/null || true
fi

# Clean up shared memory segments
echo "🧽 Cleaning shared memory..."
ipcs -m | grep $USER | awk '{print $2}' | xargs -r ipcrm -m 2>/dev/null || true

echo "✅ Cleanup complete!"
echo ""

# Show remaining processes (for verification)
echo "🔍 Checking for remaining ROS/Gazebo processes..."
remaining_ros=$(pgrep -f "ros2|gazebo|gz|rviz" 2>/dev/null || true)
remaining_project=$(pgrep -f "yolo|neural|projection|kalman|drone" 2>/dev/null || true)

if [ -n "$remaining_ros" ] || [ -n "$remaining_project" ]; then
    echo "⚠️  Some processes may still be running:"
    ps aux | grep -E "(ros2|gazebo|gz|rviz|yolo|neural|projection|kalman|drone)" | grep -v grep || true
    echo ""
    echo "💡 If processes persist, you may need to:"
    echo "   1. Restart your terminal"
    echo "   2. Log out and log back in"
    echo "   3. Reboot the system (last resort)"
else
    echo "✅ All ROS/Gazebo processes successfully terminated!"
fi

echo ""
echo "🚀 System is now clean and ready for next launch!"
echo "====================================="