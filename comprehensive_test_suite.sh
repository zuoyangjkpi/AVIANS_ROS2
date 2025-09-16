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
    # Use more specific process matching and check if process is actually running
    if pgrep -x "$process_name" > /dev/null 2>&1; then
        return 0
    elif pgrep -f "$process_name" > /dev/null 2>&1; then
        # Additional check: verify the process is actually active
        local pid=$(pgrep -f "$process_name" | head -1)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            return 0
        fi
    fi
    return 1
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
    echo "5) 🎯 Full Integration Test (Fixed System)"
    echo "6) 🚁 NMPC Person Tracking Test"
    echo "7) 🎮 NMPC + Gazebo Visual Tracking"
    echo "8) 🧹 Kill All ROS Processes"
    echo "9) 🔧 Rebuild Project"
    echo "10) 📊 Performance Monitor"
    echo "11) 🔧 System Diagnostics and Testing"
    echo "0) 🚪 Exit"
    echo ""
    read -p "Enter your choice (0-11): " choice
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
        print_status $YELLOW "⚠️  Workspace not built - run option 9"
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
    
    # Check NMPC processes
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker running"
    else
        print_status $YELLOW "⚠️  NMPC tracker not running"
    fi
    
    if check_process "nmpc_test_node"; then
        print_status $GREEN "✅ NMPC test node running"
    else
        print_status $YELLOW "⚠️  NMPC test node not running"
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
    
    # Launch in background with proper environment
    export HOME="$HOME"
    export USER="$USER"
    export DISPLAY="${DISPLAY:-:0}"
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
    
    # Clean up any existing YOLO processes automatically
    print_status $YELLOW "🧹 Cleaning up any existing YOLO processes..."
    pkill -f "yolo12_detector_node" 2>/dev/null
    sleep 2
    
    # Double check and force kill if needed
    if check_process "yolo12_detector_node"; then
        print_status $YELLOW "⚠️  Force killing stubborn YOLO processes..."
        pkill -9 -f "yolo12_detector_node" 2>/dev/null
        sleep 2
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
    
    # Start YOLO with fixed topics - ROS2 environment should be sourced via drone command
    
    ros2 run neural_network_detector yolo12_detector_node \
        --ros-args \
        -p "model_path:=$model_path" \
        -p "labels_path:=$labels_path" \
        -p "use_gpu:=false" \
        -p "confidence_threshold:=0.3" \
        -p "desired_class:=0" \
        -p "iou_threshold:=0.4" \
        -p "publish_debug_image:=true" \
        -p "max_update_rate_hz:=2.0" > /tmp/yolo.log 2>&1 &
    
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
    
    # Check NMPC topics
    if ros2 topic list | grep -q "/X3/odometry"; then
        check_topic_rate "/X3/odometry" 10.0
    fi
    
    if ros2 topic list | grep -q "/X3/cmd_vel"; then
        check_topic_rate "/X3/cmd_vel" 5.0
    fi
    
    echo ""
    print_status $YELLOW "🔍 Topic details:"
    echo "Camera topic info:"
    ros2 topic info /camera/image_raw 2>/dev/null || echo "  Not available"
    
    echo "Detection topic info:"
    ros2 topic info /person_detections 2>/dev/null || echo "  Not available"
    
    echo "NMPC Drone odometry info:"
    ros2 topic info /X3/odometry 2>/dev/null || echo "  Not available"
    
    echo "NMPC Control commands info:"
    ros2 topic info /X3/cmd_vel 2>/dev/null || echo "  Not available"
}

# Full integration test (Fixed System)
full_integration_test() {
    print_status $BLUE "🎯 Full Integration Test (Fixed System)"
    echo "========================================"
    
    print_status $YELLOW "🔄 Starting complete fixed system with enhanced human detection..."
    print_status $YELLOW "📋 Launch sequence:"
    print_status $YELLOW "   1. Gazebo simulation environment"
    print_status $YELLOW "   2. YOLO person detector (optimized parameters)"
    print_status $YELLOW "   3. Detection visualizer"
    print_status $YELLOW "   4. Drone TF publisher"
    print_status $YELLOW "   5. RViz visualization with trajectory display"
    print_status $YELLOW "   6. NMPC tracker (tracking real Gazebo walking_person)"
    print_status $YELLOW "   7. Enable tracking"
    
    # Clean up existing processes
    print_status $YELLOW "🧹 Cleaning up existing processes..."
    kill_all_processes
    sleep 3
    
    # Step 1: Launch Gazebo
    print_status $YELLOW "Step 1/8: Starting Gazebo simulation..."
    if ! launch_gazebo; then
        print_status $RED "❌ Gazebo startup failed, cannot continue"
        return 1
    fi
    
    # Step 2: Start YOLO detector
    print_status $YELLOW "Step 2/8: Starting YOLO detector..."
    if ! test_yolo_detector; then
        print_status $YELLOW "⚠️  YOLO detector had issues, but continuing..."
    fi
    
    # Start C++ detection visualizer node
    print_status $YELLOW "Step 3/8: Starting detection visualizer..."
    
    ros2 run neural_network_detector detection_visualizer_node > /tmp/detection_visualizer.log 2>&1 &
    local viz_pid=$!
    sleep 2
    
    if check_process "detection_visualizer_node"; then
        print_status $GREEN "✅ Detection visualizer started successfully"
    else
        print_status $RED "❌ Detection visualizer failed to start"
    fi
    
    # Step 4: Start drone TF publisher for proper RViz display
    print_status $YELLOW "Step 4/8: Starting drone TF publisher..."
    python3 drone_tf_publisher.py > /tmp/drone_tf.log 2>&1 &
    local tf_pid=$!
    sleep 1
    
    # Step 5: Start RViz visualization with trajectory display
    print_status $YELLOW "Step 5/8: Starting RViz visualization..."
    python3 visualization_node.py > /tmp/visualization.log 2>&1 &
    local viz_node_pid=$!
    sleep 2
    
    if check_process "visualization_node.py"; then
        print_status $GREEN "✅ RViz visualization node started successfully"
        print_status $YELLOW "💡 Open RViz2 and add these topics for visualization:"
        print_status $YELLOW "   - /person_position_markers (MarkerArray) - Red=Predicted, Blue=Actual person"
        print_status $YELLOW "   - /drone_position_markers (MarkerArray) - Green=Current drone position"
        print_status $YELLOW "   - /drone_trajectory_markers (MarkerArray)"
        print_status $YELLOW "   - /drone_path (Path)"
        print_status $YELLOW "   - /detection_image (Image) - Raw image with detection boxes"
    else
        print_status $YELLOW "⚠️  RViz visualization node failed to start (continuing anyway)"
    fi
    
    # Step 6: Start NMPC tracker (removed test_node - using real Gazebo walking_person instead)
    print_status $YELLOW "Step 6/7: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker_fixed.log 2>&1 &
    local tracker_pid=$!
    sleep 3
    
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker started successfully"
    else
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi
    
    # Step 7: Enable drone and tracking
    print_status $YELLOW "Step 7/7: Enabling drone control and tracking..."
    
    # 启用无人机控制
    print_status $YELLOW "  - 启用无人机控制..."
    ros2 topic pub -r 1 /X3/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
    local enable_drone_pid=$!
    
    # 启用NMPC跟踪
    print_status $YELLOW "  - 启用NMPC跟踪..."
    ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
    local enable_nmpc_pid=$!
    
    sleep 5
    
    # System status verification
    print_status $GREEN "🎉 Complete system startup finished!"
    print_status $BLUE "========================================"
    print_status $YELLOW "📊 System status verification:"
    
    # Check critical topics
    if wait_for_topic "/camera/image_raw" 5; then
        print_status $GREEN "  ✅ Camera images available"
    else
        print_status $RED "  ❌ Camera images not available"
    fi
    
    if wait_for_topic "/person_detections" 5; then
        print_status $GREEN "  ✅ Person detections available"
    else
        print_status $RED "  ❌ Person detections not available"
    fi
    
    if wait_for_topic "/X3/odometry" 5; then
        print_status $GREEN "  ✅ Drone odometry available"
    else
        print_status $RED "  ❌ Drone odometry not available"
    fi
    
    if wait_for_topic "/X3/cmd_vel" 5; then
        print_status $GREEN "  ✅ Drone control commands available"
    else
        print_status $RED "  ❌ Drone control commands not available"
    fi
    
    # 添加检测图像话题检查
    if wait_for_topic "/detection_image" 5; then
        print_status $GREEN "  ✅ Detection image available"
    else
        print_status $RED "  ❌ Detection image not available"
    fi
    
    print_status $BLUE "========================================"
    print_status $GREEN "🎯 System functionality verification:"
    
    # Check topic data rates
    if check_topic_rate "/person_detections" 1.0; then
        print_status $GREEN "  ✅ Person detections are being published"
    else
        print_status $YELLOW "  ⚠️  Person detection data rate is low"
    fi
    
    if check_topic_rate "/X3/odometry" 10.0; then
        print_status $GREEN "  ✅ Drone odometry is being published"
    else
        print_status $YELLOW "  ⚠️  Drone odometry data rate is low"
    fi
    
    if check_topic_rate "/X3/cmd_vel" 1.0; then
        print_status $GREEN "  ✅ NMPC is publishing control commands"
    else
        print_status $YELLOW "  ⚠️  NMPC control command data rate is low"
    fi
    
    # 添加检测图像话题数据速率检查
    if check_topic_rate "/detection_image" 1.0; then
        print_status $GREEN "  ✅ Detection images are being published"
    else
        print_status $YELLOW "  ⚠️  Detection image data rate is low"
    fi
    
    print_status $BLUE "========================================"
    print_status $GREEN "🎉 AVIANS system is running!"
    print_status $YELLOW "💡 Expected behavior:"
    print_status $YELLOW "   - See drone model in Gazebo"
    print_status $YELLOW "   - Drone should start flying and tracking walking_person in Gazebo"
    print_status $YELLOW "   - Red sphere: Drone's prediction of person position"
    print_status $YELLOW "   - Blue cylinder: Actual person position from Gazebo"
    print_status $YELLOW "   - Check logs: tail -f /tmp/nmpc_*.log"
    print_status $YELLOW "   - Use option 8 to stop all processes"
}

# NMPC Person Tracking Test
nmpc_person_tracking_test() {
    print_status $BLUE "🚁 NMPC Person Tracking Test"
    echo "============================="
    
    print_status $YELLOW "🔄 Starting NMPC person tracking system..."
    
    # Kill existing NMPC processes if running
    if check_process "nmpc_tracker_node" || check_process "nmpc_test_node"; then
        print_status $YELLOW "🛑 Stopping existing NMPC processes..."
        pkill -f "nmpc_tracker_node" 2>/dev/null
        pkill -f "nmpc_test_node" 2>/dev/null
        sleep 2
    fi
    
    # Step 1: Start test node (simulated person)
    print_status $YELLOW "Step 1: Starting person simulator..."
    ros2 run drone_nmpc_tracker nmpc_test_node > /tmp/nmpc_test.log 2>&1 &
    local test_pid=$!
    sleep 3
    
    if check_process "nmpc_test_node"; then
        print_status $GREEN "✅ Person simulator started"
        
        # Wait for person detection topic
        if wait_for_topic "/person_detections" 5; then
            print_status $GREEN "✅ Person detections available"
        else
            print_status $RED "❌ Person detections not available"
            return 1
        fi
    else
        print_status $RED "❌ Person simulator failed to start"
        return 1
    fi
    
    # Step 2: Start NMPC tracker
    print_status $YELLOW "Step 2: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker.log 2>&1 &
    local tracker_pid=$!
    sleep 3
    
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker started"
        
        # Wait for control commands
        if wait_for_topic "/X3/cmd_vel" 5; then
            print_status $GREEN "✅ NMPC control commands available"
        else
            print_status $YELLOW "⚠️  NMPC control commands not yet available"
        fi
    else
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi
    
    # Step 3: Enable tracking
    print_status $YELLOW "Step 3: Enabling tracking..."
    ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
    local enable_pid=$!
    sleep 2
    
    # Step 4: Monitor performance
    print_status $YELLOW "Step 4: Monitoring NMPC performance..."
    
    # Check person detection rate
    print_status $YELLOW "🔍 Checking person detection rate..."
    if check_topic_rate "/person_detections" 5.0; then
        print_status $GREEN "✅ Person detections at good rate"
    else
        print_status $YELLOW "⚠️  Person detection rate lower than expected"
    fi
    
    # Check drone odometry
    print_status $YELLOW "🔍 Checking drone odometry..."
    if ros2 topic list | grep -q "/X3/odometry"; then
        if check_topic_rate "/X3/odometry" 10.0; then
            print_status $GREEN "✅ Drone odometry at good rate"
        else
            print_status $YELLOW "⚠️  Drone odometry rate lower than expected"
        fi
    fi
    
    # Check control commands
    print_status $YELLOW "🔍 Checking NMPC control output..."
    if check_topic_rate "/X3/cmd_vel" 5.0; then
        print_status $GREEN "✅ NMPC generating control commands"
        
        # Show a sample control command
        print_status $YELLOW "📊 Sample control command:"
        timeout 3s ros2 topic echo /X3/cmd_vel --once 2>/dev/null || echo "  No data available"
    else
        print_status $YELLOW "⚠️  NMPC control output rate lower than expected"
    fi
    
    print_status $GREEN "🎉 NMPC Person Tracking Test completed!"
    print_status $YELLOW "💡 Your drone is actively tracking the simulated person!"
    print_status $YELLOW "💡 Check logs: tail /tmp/nmpc_tracker.log"
    print_status $YELLOW "💡 Press Ctrl+C in a terminal to stop processes"
}

# System Diagnostics and Testing
system_diagnostics_test() {
    print_status $BLUE "🔧 System Diagnostics and Testing"
    echo "=================================="
    
    # Function to check if a topic is publishing
    check_topic_diagnostics() {
        local topic=$1
        print_status $YELLOW "Checking topic: $topic"
        
        if ros2 topic list | grep -q "$topic"; then
            print_status $GREEN "  ✅ Topic $topic exists"
            
            # Check if data is being published
            local data=$(timeout 3s ros2 topic echo "$topic" --once 2>/dev/null)
            if [ -n "$data" ]; then
                print_status $GREEN "  ✅ Topic $topic has data"
                return 0
            else
                print_status $YELLOW "  ⚠️  Topic $topic exists but no data"
                return 1
            fi
        else
            print_status $RED "  ❌ Topic $topic not found"
            return 1
        fi
    }

    # Check if ROS2 is sourced
    if [ -z "$ROS_DISTRO" ]; then
        print_status $RED "❌ ROS2 not sourced"
        return 1
    fi

    print_status $GREEN "✅ ROS2 $ROS_DISTRO environment active"

    # Test 1: Check visualization topics
    print_status $BLUE "Test 1: Checking visualization topics"
    check_topic_diagnostics "/person_position_markers"
    check_topic_diagnostics "/drone_trajectory_markers" 
    check_topic_diagnostics "/drone_path"

    # Test 2: Check YOLO topics
    print_status $BLUE "Test 2: Checking YOLO detection topics"
    check_topic_diagnostics "/person_detections"
    check_topic_diagnostics "/detection_image"

    # Test 3: Check drone topics
    print_status $BLUE "Test 3: Checking drone topics"
    check_topic_diagnostics "/X3/odometry"
    check_topic_diagnostics "/X3/cmd_vel"

    # Test 4: Check camera topic
    print_status $BLUE "Test 4: Checking camera topic"
    check_topic_diagnostics "/camera/image_raw"

    # Test 5: Check running processes
    print_status $BLUE "Test 5: Checking running processes"
    
    if check_process "gz sim"; then
        print_status $GREEN "  ✅ Gazebo simulation running"
    else
        print_status $YELLOW "  ⚠️  Gazebo not running"
    fi
    
    if check_process "yolo12_detector_node"; then
        print_status $GREEN "  ✅ YOLO detector running"
    else
        print_status $YELLOW "  ⚠️  YOLO detector not running"
    fi
    
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "  ✅ NMPC tracker running"
    else
        print_status $YELLOW "  ⚠️  NMPC tracker not running"
    fi
    
    if check_process "nmpc_test_node"; then
        print_status $GREEN "  ✅ NMPC test node running"
    else
        print_status $YELLOW "  ⚠️  NMPC test node not running"
    fi

    print_status $GREEN "🎉 Diagnostics completed!"
    print_status $YELLOW "💡 If topics show 'no data', make sure the simulation is running"
    print_status $YELLOW "💡 Run option 5 to start full integration test"
}

# NMPC + Gazebo Visual Tracking
nmpc_gazebo_visual_tracking() {
    print_status $BLUE "🎮 NMPC + Gazebo Visual Tracking"
    echo "=================================="
    
    # Check if Gazebo packages are available
    if ! command -v gz > /dev/null 2>&1; then
        print_status $RED "❌ Gazebo not installed!"
        print_status $YELLOW "💡 Install with: sudo apt install gz-garden"
        print_status $YELLOW "💡 Also install: sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim"
        return 1
    fi
    
    print_status $YELLOW "🔄 Starting complete visual tracking system..."
    
    # Step 1: Clean up existing processes
    print_status $YELLOW "Step 1: Cleaning up existing processes..."
    pkill -f "gz sim" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    pkill -f "nmpc_test_node" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    sleep 3
    
    # Step 2: Start Gazebo
    print_status $YELLOW "Step 2: Starting Gazebo simulation..."
    if ! launch_gazebo; then
        print_status $RED "❌ Failed to start Gazebo"
        return 1
    fi
    
    # Step 3: Start person simulator
    print_status $YELLOW "Step 3: Starting person simulator..."
    ros2 run drone_nmpc_tracker nmpc_test_node > /tmp/nmpc_test_gazebo.log 2>&1 &
    sleep 3
    
    if ! check_process "nmpc_test_node"; then
        print_status $RED "❌ Person simulator failed to start"
        return 1
    fi
    print_status $GREEN "✅ Person simulator running"
    
    # Step 4: Start NMPC tracker
    print_status $YELLOW "Step 4: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker_gazebo.log 2>&1 &
    sleep 3
    
    if ! check_process "nmpc_tracker_node"; then
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi
    print_status $GREEN "✅ NMPC tracker running"
    
    # Step 5: Enable tracking
    print_status $YELLOW "Step 5: Enabling tracking..."
    ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
    sleep 2
    
    # Step 6: Launch RViz for visualization (optional)
    print_status $YELLOW "Step 6: Starting RViz for additional visualization..."
    if command -v rviz2 > /dev/null 2>&1; then
        rviz2 > /dev/null 2>&1 &
        print_status $GREEN "✅ RViz started"
    else
        print_status $YELLOW "⚠️  RViz not available"
    fi
    
    print_status $GREEN "🎉 Complete NMPC + Gazebo system is running!"
    print_status $YELLOW "💡 You should now see:"
    print_status $YELLOW "   - Gazebo window with drone simulation"
    print_status $YELLOW "   - Drone tracking simulated person movement"
    print_status $YELLOW "   - RViz showing tracking data (if available)"
    print_status $YELLOW "💡 Use option 8 to stop all processes"
}

# Kill all ROS processes
kill_all_processes() {
    print_status $BLUE "🧹 Killing All ROS Processes"
    echo "============================="
    
    print_status $YELLOW "🛑 Stopping all processes..."
    
    # Kill specific processes
    pkill -f "gz sim" 2>/dev/null
    pkill -f "yolo12_detector_node" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    pkill -f "nmpc_test_node" 2>/dev/null
    pkill -f "detection_visualizer_node" 2>/dev/null
    pkill -f "visualization_node.py" 2>/dev/null
    pkill -f "drone_tf_publisher.py" 2>/dev/null
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "ros2 topic pub" 2>/dev/null
    
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
    
    if check_process "nmpc_tracker_node"; then
        print_status $YELLOW "🚁 NMPC performance:"
        echo "  NMPC optimization warnings are normal"
        # Check NMPC control commands
        if ros2 topic list | grep -q "/X3/cmd_vel"; then
            print_status $GREEN "  ✅ NMPC publishing control commands"
        else
            print_status $YELLOW "  ⚠️  No NMPC control commands"
        fi
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
            6) nmpc_person_tracking_test ;;
            7) nmpc_gazebo_visual_tracking ;;
            8) kill_all_processes ;;
            9) rebuild_project ;;
            10) performance_monitor ;;
            11) system_diagnostics_test ;;
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