#!/bin/bash

# YOLO Timeout Test
# ================

echo "🕐 YOLO Timeout Test"
echo "==================="
echo "This will run YOLO with a timeout to catch hanging issues"
echo ""

# Set timeout (30 seconds)
TIMEOUT=30

echo "Starting YOLO detector with ${TIMEOUT}s timeout..."
echo "If it hangs, we'll know it's a performance issue"
echo ""

# Find model and labels
MODEL_PATH="$(find . -name "yolo12n.onnx" | head -1)"
LABELS_PATH="$(find . -name "coco.names" | head -1)"

if [ -z "$MODEL_PATH" ] || [ -z "$LABELS_PATH" ]; then
    echo "❌ Model or labels not found"
    exit 1
fi

echo "Using:"
echo "  Model: $MODEL_PATH"
echo "  Labels: $LABELS_PATH"
echo ""

# Run YOLO with timeout
timeout $TIMEOUT ros2 run neural_network_detector yolo12_detector_node \
    --ros-args \
    -p "model_path:=$MODEL_PATH" \
    -p "labels_path:=$LABELS_PATH" \
    -p "use_gpu:=false" \
    -p "confidence_threshold:=0.5" \
    -p "desired_class:=0" \
    -p "max_update_rate_hz:=1.0" \
    --log-level INFO

EXIT_CODE=$?

echo ""
echo "📋 Results:"
if [ $EXIT_CODE -eq 124 ]; then
    echo "⏰ YOLO timed out after ${TIMEOUT} seconds"
    echo "💡 This confirms it's hanging - likely CPU performance issue"
    echo ""
    echo "🔧 Solutions:"
    echo "1. Reduce YOLO input resolution"
    echo "2. Lower processing frequency"
    echo "3. Use GPU if available"
    echo "4. Try a smaller YOLO model (yolo8n instead of yolo12n)"
elif [ $EXIT_CODE -eq 0 ]; then
    echo "✅ YOLO completed successfully"
elif [ $EXIT_CODE -eq 130 ]; then
    echo "🛑 YOLO stopped by user (Ctrl+C)"
else
    echo "❌ YOLO failed with exit code: $EXIT_CODE"
fi
