#!/bin/bash

# AVIANS ROS2 PORT1 - Fix YOLO Model Paths
# ========================================

echo "🔧 AVIANS ROS2 PORT1 - YOLO Path Fixer"
echo "======================================"

# Check if we're in the right directory
if [ ! -d "src/neural_network_detector" ]; then
    echo "❌ Error: neural_network_detector not found"
    echo "Please run this script from the workspace root (AVIANS_ROS2_PORT1)"
    exit 1
fi

echo "✅ Workspace found"

# Find actual model locations
echo ""
echo "🔍 Searching for YOLO model files..."

# Search for yolo models
YOLO_MODELS=$(find . -name "*.onnx" -path "*/neural_network_detector/*" 2>/dev/null)
COCO_LABELS=$(find . -name "coco.names" -path "*/neural_network_detector/*" 2>/dev/null)

echo "Found YOLO models:"
if [ -n "$YOLO_MODELS" ]; then
    echo "$YOLO_MODELS" | while read model; do
        size=$(du -h "$model" 2>/dev/null | cut -f1)
        echo "   ✅ $model ($size)"
    done
else
    echo "   ❌ No YOLO models found"
fi

echo ""
echo "Found COCO labels:"
if [ -n "$COCO_LABELS" ]; then
    echo "$COCO_LABELS" | while read labels; do
        lines=$(wc -l < "$labels" 2>/dev/null)
        echo "   ✅ $labels ($lines classes)"
    done
else
    echo "   ❌ No COCO labels found"
fi

# Get the first (best) model and labels
BEST_MODEL=$(echo "$YOLO_MODELS" | head -n1)
BEST_LABELS=$(echo "$COCO_LABELS" | head -n1)

if [ -z "$BEST_MODEL" ]; then
    echo ""
    echo "❌ No YOLO model found! Please check:"
    echo "   1. Model should be in: src/neural_network_detector/third_party/models/"
    echo "   2. Or in: src/neural_network_detector/third_party/YOLOs-CPP/models/"
    echo "   3. File should have .onnx extension"
    exit 1
fi

if [ -z "$BEST_LABELS" ]; then
    echo ""
    echo "⚠️  No COCO labels found, creating default ones..."
    LABELS_DIR=$(dirname "$BEST_MODEL")
    BEST_LABELS="$LABELS_DIR/coco.names"
    
    # Create basic COCO labels
    cat > "$BEST_LABELS" << 'EOF'
person
bicycle
car
motorbike
aeroplane
bus
train
truck
boat
traffic light
fire hydrant
stop sign
parking meter
bench
bird
cat
dog
horse
sheep
cow
elephant
bear
zebra
giraffe
backpack
umbrella
handbag
tie
suitcase
frisbee
skis
snowboard
sports ball
kite
baseball bat
baseball glove
skateboard
surfboard
tennis racket
bottle
wine glass
cup
fork
knife
spoon
bowl
banana
apple
sandwich
orange
broccoli
carrot
hot dog
pizza
donut
cake
chair
sofa
pottedplant
bed
diningtable
toilet
tvmonitor
laptop
mouse
remote
keyboard
cell phone
microwave
oven
toaster
sink
refrigerator
book
clock
vase
scissors
teddy bear
hair drier
toothbrush
EOF
    echo "   ✅ Created COCO labels: $BEST_LABELS"
fi

# Convert to absolute paths
ABSOLUTE_MODEL=$(realpath "$BEST_MODEL")
ABSOLUTE_LABELS=$(realpath "$BEST_LABELS")

echo ""
echo "📁 Using files:"
echo "   Model:  $ABSOLUTE_MODEL"
echo "   Labels: $ABSOLUTE_LABELS"

# Create a test script with correct paths
echo ""
echo "🎯 Creating YOLO test script with correct paths..."

cat > test_yolo_fixed.py << EOF
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
    model_path = "$ABSOLUTE_MODEL"
    labels_path = "$ABSOLUTE_LABELS"
    
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
        '-p', 'use_gpu:=false',
        '-p', 'confidence_threshold:=0.3',
        '-p', 'desired_class:=0',  # Person class
        '-p', 'publish_debug_image:=true',
        '-p', 'max_update_rate_hz:=2.0'
    ]
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\\n🛑 Stopped")

if __name__ == "__main__":
    main()
EOF

chmod +x test_yolo_fixed.py
echo "   ✅ Created: test_yolo_fixed.py"

# Update launch file if it exists
LAUNCH_FILE="src/drone_state_publisher/launch/simulation.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo ""
    echo "🚀 Updating launch file with correct paths..."
    
    # Create backup
    cp "$LAUNCH_FILE" "${LAUNCH_FILE}.backup"
    echo "   ✅ Backup created: ${LAUNCH_FILE}.backup"
    
    # Update paths in launch file
    sed -i "s|default_yolo_model_path = .*|default_yolo_model_path = \"$ABSOLUTE_MODEL\"|" "$LAUNCH_FILE"
    sed -i "s|default_yolo_labels_path = .*|default_yolo_labels_path = \"$ABSOLUTE_LABELS\"|" "$LAUNCH_FILE"
    
    echo "   ✅ Updated launch file paths"
fi

# Create a simple launcher with correct paths
echo ""
echo "🚀 Creating simple launcher..."

cat > launch_yolo_simple.sh << 'EOF'
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
EOF

chmod +x launch_yolo_simple.sh
echo "   ✅ Created: launch_yolo_simple.sh"

echo ""
echo "✅ YOLO paths fixed!"
echo ""
echo "🚀 Next steps:"
echo "1. Test YOLO detector:"
echo "   ./test_yolo_fixed.py"
echo ""
echo "2. Or use simple launcher:"
echo "   ./launch_yolo_simple.sh"
echo ""
echo "3. For full simulation:"
echo "   ros2 launch drone_description gz.launch.py"
echo "   # Then in another terminal:"
echo "   ./test_yolo_fixed.py"
echo ""
echo "📁 Files created:"
echo "   - test_yolo_fixed.py      (YOLO test with correct paths)"
echo "   - launch_yolo_simple.sh   (Simple launcher)"
if [ -f "${LAUNCH_FILE}.backup" ]; then
    echo "   - ${LAUNCH_FILE}.backup (Launch file backup)"
fi