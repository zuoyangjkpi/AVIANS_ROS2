#!/bin/bash

# YOLO Lightweight Parameters Test
# ================================

echo "🪶 YOLO Lightweight Parameters Test"
echo "==================================="
echo "Testing YOLO with CPU-optimized parameters"
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

echo "🎯 Optimized Parameters for CPU:"
echo "  - Lower input resolution (320x320 instead of 640x640)"
echo "  - Reduced processing rate (0.5 Hz instead of 2 Hz)"
echo "  - Higher confidence threshold (0.7 instead of 0.3)"
echo "  - Disabled debug images"
echo ""

echo "Starting optimized YOLO detector..."
echo "Press Ctrl+C to stop"
echo ""

ros2 run neural_network_detector yolo12_detector_node \
    --ros-args \
    -p "model_path:=$MODEL_PATH" \
    -p "labels_path:=$LABELS_PATH" \
    -p "use_gpu:=false" \
    -p "confidence_threshold:=0.7" \
    -p "desired_class:=0" \
    -p "desired_width:=320" \
    -p "desired_height:=320" \
    -p "max_update_rate_hz:=0.5" \
    -p "publish_debug_image:=false" \
    --log-level INFO
