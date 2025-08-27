#!/bin/bash

# Fix Topic Mapping for YOLO Detector
# ===================================

echo "🔧 Fixing YOLO topic mapping..."
echo "================================"

# Navigate to project directory
cd "$(dirname "$0")"

# Check if we're in the right directory
if [ ! -f "src/neural_network_detector/src/yolo12_detector_node.cpp" ]; then
    echo "❌ Error: Not in the correct project directory"
    echo "   Please run this script from the project root directory"
    exit 1
fi

# Backup original file
echo "📋 Creating backup..."
cp src/neural_network_detector/src/yolo12_detector_node.cpp src/neural_network_detector/src/yolo12_detector_node.cpp.backup

# Fix the topic subscription
echo "🔄 Fixing image topic subscription..."
sed -i 's/"image_raw"/"camera\/image_raw"/g' src/neural_network_detector/src/yolo12_detector_node.cpp

# Also fix the output topics to be more descriptive
echo "🔄 Fixing output topic names..."
sed -i 's/"detections"/"person_detections"/g' src/neural_network_detector/src/yolo12_detector_node.cpp
sed -i 's/"detection_count"/"person_detection_count"/g' src/neural_network_detector/src/yolo12_detector_node.cpp

# Check if changes were made
if grep -q "camera/image_raw" src/neural_network_detector/src/yolo12_detector_node.cpp; then
    echo "✅ Successfully updated image topic: image_raw → camera/image_raw"
else
    echo "❌ Failed to update image topic"
fi

if grep -q "person_detections" src/neural_network_detector/src/yolo12_detector_node.cpp; then
    echo "✅ Successfully updated detection topic: detections → person_detections"
else
    echo "❌ Failed to update detection topic"
fi

echo ""
echo "📝 Changes made:"
echo "  Input:  image_raw → camera/image_raw"
echo "  Output: detections → person_detections"
echo "  Output: detection_count → person_detection_count"
echo ""
echo "💾 Backup saved as: yolo12_detector_node.cpp.backup"
echo ""
echo "🔨 Next steps:"
echo "1. Rebuild the project: colcon build --packages-select neural_network_detector"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Test YOLO: ros2 run neural_network_detector yolo12_detector_node"
echo ""
echo "✅ Topic mapping fix complete!"
