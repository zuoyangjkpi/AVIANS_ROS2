#!/bin/bash

# YOLO Debug Launch with Verbose Logging
# ======================================

echo "🔍 YOLO Debug Launch"
echo "==================="

# Find model and labels
MODEL_PATH="$(find . -name "yolo12n.onnx" | head -1)"
LABELS_PATH="$(find . -name "coco.names" | head -1)"

echo "Using:"
echo "  Model: $MODEL_PATH"
echo "  Labels: $LABELS_PATH"
echo ""

echo "🚀 Starting YOLO with debug logging..."
echo "Watch for where it hangs:"
echo ""

# Set debug environment
export RCUTILS_LOGGING_SEVERITY=DEBUG
export RCUTILS_COLORIZED_OUTPUT=1

# Run with strace to see system calls (optional)
# strace -e trace=read,write,poll,select,epoll_wait -o yolo_strace.log \

ros2 run neural_network_detector yolo12_detector_node \
    --ros-args \
    -p "model_path:=$MODEL_PATH" \
    -p "labels_path:=$LABELS_PATH" \
    -p "use_gpu:=false" \
    -p "confidence_threshold:=0.7" \
    -p "desired_class:=0" \
    -p "max_update_rate_hz:=0.5" \
    -p "publish_debug_image:=false" \
    --log-level DEBUG \
    --ros-args --log-level rcl:=DEBUG
