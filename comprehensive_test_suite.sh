#!/bin/bash

# AVIANS ROS2 PORT1 - Comprehensive Test Suite
# ============================================

echo "🚀 AVIANS ROS2 PORT1 - Comprehensive Test Suite"
echo "================================================"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to check if a process is running
check_process() {
    local process_name=$1
    if pgrep -f "$process_name" > /dev/null; then
        return 0
    else
        return 1
    fi
}

# Function to wait for topic to be available
wait_for_topic() {
    local topic_name=$1
    local timeout=${2:-10}
    local count=0
    
    print_status $YELLOW "⏳ Waiting for topic: $topic_name"
    
    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "$topic_name"; then
            print_status $GREEN "✅ Topic $topic_name is available"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    
    print_status $RED "❌ Timeout waiting for topic: $topic_name"
    return 1
}

# Function to check topic data rate
check_topic_rate() {
    local topic_name=$1
    local min_rate=${2:-1.0}
    
    print_status $YELLOW "📊 Checking data rate for: $topic_name"
    
    # Use timeout to limit the check to 5 seconds
    local rate_output=$(timeout 5s ros2 topic hz "$topic_name" 2>/dev/null | tail -1)
    
    if [[ $rate_output == *"average rate"* ]]; then
        local rate=$(echo "$rate_output" | grep -o '[0-9]*\.[0-9]*' | head -1)
        if (( $(echo "$rate >= $min_rate" | bc -l) )); then
            print_status $GREEN "✅ Topic $topic_name rate: ${rate} Hz (>= ${min_rate} Hz)"
            return 0
        else
            print_status $YELLOW "⚠️  Topic $topic_name rate: ${rate} Hz (< ${min_rate} Hz)"
            return 1
        fi
    else
        print_status $RED "❌ No data on topic: $topic_name"
        return 1
    fi
}

# Main menu function
show_menu() {
    echo ""
    print_status $BLUE "📋 Test Options:"
    echo "1) 🔍 System Status Check"
    echo "2) 🎮 Launch Gazebo Simulation"
    echo "3) 🧠 Test YOLO Detector (Fixed Topics)"
    echo "4) 📡 Monitor All Topics"
    echo "5) 🎯 Full Integration Test"
    echo "6) 🧹 Kill All ROS Processes"
    echo "7) 🔧 Rebuild Project"
    echo "8) 📊 Performance Monitor"
    echo "0) 🚪 Exit"
    echo ""
    read -p "Enter your choice (0-8): " choice
}

# System status check
system_status_check() {
    print_status $BLUE "🔍 System Status Check"
    echo "======================"
    
    # Check ROS2 environment
    if [ -z "$ROS_DISTRO" ]; then
        print_status $RED "❌ ROS2 not sourced"
        return 1
    else
        print_status $GREEN "✅ ROS2 $ROS_DISTRO environment active"
    fi
    
    # Check workspace
    if [ -f "install/setup.bash" ]; then
        print_status $GREEN "✅ Workspace built"
    else
        print_status $YELLOW "⚠️  Workspace not built - run option 7"
    fi
    
    # Check key processes
    if check_process "gz sim"; then
        print_status $GREEN "✅ Gazebo simulation running"
    else
        print_status $YELLOW "⚠️  Gazebo not running"
    fi
    
    if check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ YOLO detector running"
    else
        print_status $YELLOW "⚠️  YOLO detector not running"
    fi
    
    # Check topics
    local topic_count=$(ros2 topic list 2>/dev/null | wc -l)
    print_status $GREEN "📡 Active topics: $topic_count"
    
    # Check specific topics
    if ros2 topic list | grep -q "/camera/image_raw"; then
        print_status $GREEN "✅ Camera topic available"
    else
        print_status $YELLOW "⚠️  Camera topic not available"
    fi
    
    if ros2 topic list | grep -q "/person_detections"; then
        print_status $GREEN "✅ Detection topic available"
    else
        print_status $YELLOW "⚠️  Detection topic not available"
    fi
}

# Launch Gazebo simulation
launch_gazebo() {
    print_status $BLUE "🎮 Launching Gazebo Simulation"
    echo "==============================="
    
    if check_process "gz sim"; then
        print_status $YELLOW "⚠️  Gazebo already running"
        read -p "Kill existing Gazebo? (y/n): " kill_gazebo
        if [ "$kill_gazebo" = "y" ]; then
            pkill -f "gz sim"
            sleep 2
        else
            return 0
        fi
    fi
    
    print_status $YELLOW "🚀 Starting Gazebo..."
    print_status $YELLOW "   This may take 10-15 seconds..."
    
    # Launch in background
    ros2 launch drone_description gz.launch.py > /tmp/gazebo.log 2>&1 &
    local gazebo_pid=$!
    
    # Wait for Gazebo to start
    local count=0
    while [ $count -lt 30 ]; do
        if check_process "gz sim"; then
            print_status $GREEN "✅ Gazebo started successfully"
            
            # Wait for camera topic
            if wait_for_topic "/camera/image_raw" 15; then
                print_status $GREEN "✅ Camera topic is publishing"
                return 0
            else
                print_status $YELLOW "⚠️  Camera topic not yet available"
                return 1
            fi
        fi
        sleep 1
        count=$((count + 1))
    done
    
    print_status $RED "❌ Gazebo failed to start"
    return 1
}

# Test YOLO detector
test_yolo_detector() {
    print_status $BLUE "🧠 Testing YOLO Detector"
    echo "========================="
    
    # Check if Gazebo is running
    if ! check_process "gz sim"; then
        print_status $YELLOW "⚠️  Gazebo not running - starting it first..."
        if ! launch_gazebo; then
            print_status $RED "❌ Cannot test YOLO without Gazebo"
            return 1
        fi
    fi
    
    # Check if YOLO is already running
    if check_process "yolo12_detector_node"; then
        print_status $YELLOW "⚠️  YOLO already running"
        read -p "Kill existing YOLO? (y/n): " kill_yolo
        if [ "$kill_yolo" = "y" ]; then
            pkill -f "yolo12_detector_node"
            sleep 2
        else
            return 0
        fi
    fi
    
    # Find model files
    local model_path=$(find . -name "yolo12n.onnx" | head -1)
    local labels_path=$(find . -name "coco.names" | head -1)
    
    if [ -z "$model_path" ] || [ -z "$labels_path" ]; then
        print_status $RED "❌ YOLO model files not found"
        return 1
    fi
    
    print_status $GREEN "📁 Using model: $model_path"
    print_status $GREEN "📁 Using labels: $labels_path"
    
    print_status $YELLOW "🚀 Starting YOLO detector..."
    
    # Start YOLO with fixed topics
    ros2 run neural_network_detector yolo12_detector_node \
        --ros-args \
        -p "model_path:=$model_path" \
        -p "labels_path:=$labels_path" \
        -p "use_gpu:=false" \
        -p "confidence_threshold:=0.5" \
        -p "desired_class:=0" \
        -p "max_update_rate_hz:=1.0" > /tmp/yolo.log 2>&1 &
    
    local yolo_pid=$!
    
    # Wait for YOLO to start
    sleep 5
    
    if check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ YOLO detector started"
        
        # Wait for detection topic
        if wait_for_topic "/person_detections" 10; then
            print_status $GREEN "✅ Detection topic is available"
            
            # Check if detections are being published
            print_status $YELLOW "🔍 Checking for detections..."
            if check_topic_rate "/person_detections" 0.1; then
                print_status $GREEN "✅ YOLO is publishing detections!"
            else
                print_status $YELLOW "⚠️  No detections yet (this is normal if no people in view)"
            fi
            
            return 0
        else
            print_status $RED "❌ Detection topic not available"
            return 1
        fi
    else
        print_status $RED "❌ YOLO detector failed to start"
        print_status $YELLOW "📋 Check log: tail /tmp/yolo.log"
        return 1
    fi
}

# Monitor all topics
monitor_topics() {
    print_status $BLUE "📡 Topic Monitor"
    echo "================"
    
    print_status $YELLOW "📋 All active topics:"
    ros2 topic list
    
    echo ""
    print_status $YELLOW "📊 Key topic rates:"
    
    # Check camera topic
    if ros2 topic list | grep -q "/camera/image_raw"; then
        check_topic_rate "/camera/image_raw" 1.0
    fi
    
    # Check detection topic
    if ros2 topic list | grep -q "/person_detections"; then
        check_topic_rate "/person_detections" 0.1
    fi
    
    echo ""
    print_status $YELLOW "🔍 Topic details:"
    echo "Camera topic info:"
    ros2 topic info /camera/image_raw 2>/dev/null || echo "  Not available"
    
    echo "Detection topic info:"
    ros2 topic info /person_detections 2>/dev/null || echo "  Not available"
}

# Full integration test
full_integration_test() {
    print_status $BLUE "🎯 Full Integration Test"
    echo "========================"
    
    print_status $YELLOW "🔄 Running complete system test..."
    
    # Step 1: System check
    print_status $YELLOW "Step 1: System Status"
    system_status_check
    
    # Step 2: Launch Gazebo
    print_status $YELLOW "Step 2: Gazebo Simulation"
    if ! launch_gazebo; then
        print_status $RED "❌ Integration test failed at Gazebo"
        return 1
    fi
    
    # Step 3: Test YOLO
    print_status $YELLOW "Step 3: YOLO Detector"
    if ! test_yolo_detector; then
        print_status $RED "❌ Integration test failed at YOLO"
        return 1
    fi
    
    # Step 4: Monitor system
    print_status $YELLOW "Step 4: System Monitoring"
    monitor_topics
    
    print_status $GREEN "🎉 Integration test completed successfully!"
    print_status $YELLOW "💡 Your system is ready for person detection and tracking!"
}

# Kill all ROS processes
kill_all_processes() {
    print_status $BLUE "🧹 Killing All ROS Processes"
    echo "============================="
    
    print_status $YELLOW "🛑 Stopping all processes..."
    
    # Kill specific processes
    pkill -f "gz sim" 2>/dev/null
    pkill -f "yolo12_detector_node" 2>/dev/null
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    
    sleep 2
    
    # Check if processes are stopped
    if ! check_process "gz sim" && ! check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ All processes stopped"
    else
        print_status $YELLOW "⚠️  Some processes may still be running"
    fi
    
    # Clean up shared memory
    rm -f /dev/shm/sem.* 2>/dev/null
    rm -f /tmp/*.log 2>/dev/null
    
    print_status $GREEN "🧹 Cleanup complete"
}

# Rebuild project
rebuild_project() {
    print_status $BLUE "🔧 Rebuilding Project"
    echo "====================="
    
    print_status $YELLOW "🧹 Cleaning build artifacts..."
    rm -rf build/ install/ log/
    
    print_status $YELLOW "🔨 Building project..."
    if colcon build; then
        print_status $GREEN "✅ Build successful"
        print_status $YELLOW "💡 Don't forget to source: source install/setup.bash"
    else
        print_status $RED "❌ Build failed"
        return 1
    fi
}

# Performance monitor
performance_monitor() {
    print_status $BLUE "📊 Performance Monitor"
    echo "======================"
    
    print_status $YELLOW "🔍 System resources:"
    echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    echo "Memory: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
    
    print_status $YELLOW "🎯 ROS2 processes:"
    ps aux | grep -E "(gz|yolo|ros2)" | grep -v grep | while read line; do
        echo "  $line"
    done
    
    if check_process "yolo12_detector_node"; then
        print_status $YELLOW "🧠 YOLO performance:"
        echo "  Check /tmp/yolo.log for inference times"
        tail -5 /tmp/yolo.log 2>/dev/null | grep "Inference latency" || echo "  No recent inference data"
    fi
}

# Main execution
main() {
    # Check if we're in the right directory
    if [ ! -d "src" ]; then
        print_status $RED "❌ Error: Please run this script from the project root directory"
        exit 1
    fi
    
    # Source ROS2 if available
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source workspace if available
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
    
    while true; do
        show_menu
        
        case $choice in
            1) system_status_check ;;
            2) launch_gazebo ;;
            3) test_yolo_detector ;;
            4) monitor_topics ;;
            5) full_integration_test ;;
            6) kill_all_processes ;;
            7) rebuild_project ;;
            8) performance_monitor ;;
            0) 
                print_status $GREEN "👋 Goodbye!"
                exit 0
                ;;
            *)
                print_status $RED "❌ Invalid option. Please try again."
                ;;
        esac
        
        echo ""
        read -p "Press Enter to continue..."
    done
}

# Run main function
main

