#!/bin/bash

echo "🔧 测试Odometry修复"
echo "=================="
echo ""

# 设置环境
source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "❌ 工作区未构建"
    exit 1
fi

# 启动Gazebo
echo "🚀 启动Gazebo..."
ros2 launch drone_description gz.launch.py > /tmp/test_gazebo.log 2>&1 &
gazebo_pid=$!

# 等待15秒让系统完全启动
echo "⏳ 等待15秒让系统稳定..."
sleep 15

echo "📋 检查话题状态："
echo ""

echo "1. Gazebo内部话题："
timeout 3s gz topic -l | grep -E "(odometry|pose)" || echo "  无odometry话题"

echo ""
echo "2. ROS2话题："
timeout 3s ros2 topic list | grep -E "(odometry|pose)" || echo "  无odometry话题"

echo ""
echo "3. 检查Gazebo原生里程计话题："
if timeout 3s gz topic -l | grep -q "/model/X3/odometry"; then
    echo "✅ Gazebo正在发布/model/X3/odometry"
else
    echo "❌ 未找到/model/X3/odometry"
fi

echo ""
echo "4. 检查ROS2侧的/X3/odometry话题："
if ros2 topic list | grep -q "/X3/odometry"; then
    echo "✅ /X3/odometry话题存在"
    ros2 topic info /X3/odometry
    echo "获取一条消息："
    timeout 5s ros2 topic echo /X3/odometry --once
else
    echo "❌ /X3/odometry话题不存在"
fi

echo ""
echo "🎯 测试完成！按Ctrl+C退出"
wait
