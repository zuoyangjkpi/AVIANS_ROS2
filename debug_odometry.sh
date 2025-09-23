#!/bin/bash

# Odometry Debug Script
# 用于诊断X3无人机位姿数据获取问题

echo "🔍 Odometry Debug Script"
echo "======================="
echo ""

# 设置环境
echo "⚙️ 设置ROS2环境..."
source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ 工作区已源设置"
else
    echo "❌ 工作区未构建"
    exit 1
fi

# 清理现有进程
echo ""
echo "🧹 清理现有进程..."
pkill -f "gz sim" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
sleep 3

echo ""
echo "🚀 启动Gazebo仿真..."
# 启动Gazebo仿真
ros2 launch drone_description gz.launch.py > /tmp/gazebo_debug.log 2>&1 &
gazebo_pid=$!

# 等待Gazebo启动
echo "⏳ 等待Gazebo启动（最多30秒）..."
count=0
while [ $count -lt 30 ]; do
    if pgrep -f "gz sim" > /dev/null; then
        echo "✅ Gazebo已启动"
        break
    fi
    sleep 1
    count=$((count + 1))
done

if [ $count -ge 30 ]; then
    echo "❌ Gazebo启动超时"
    exit 1
fi

echo ""
echo "⏳ 等待5秒让系统稳定..."
sleep 5

echo ""
echo "📋 1. 检查Gazebo话题..."
echo "Gazebo内部话题列表："
timeout 5s gz topic -l | grep -E "(odometry|pose)" || echo "  没有找到odometry相关话题"

echo ""
echo "📋 2. 检查ROS2话题..."
echo "ROS2话题列表："
timeout 5s ros2 topic list | grep -E "(odometry|pose)" || echo "  没有找到odometry相关话题"

echo ""
echo "📋 3. 检查桥接进程..."
if pgrep -f "parameter_bridge" > /dev/null; then
    echo "✅ parameter_bridge进程正在运行"
    echo "parameter_bridge进程详细信息："
    ps aux | grep parameter_bridge | grep -v grep
else
    echo "❌ parameter_bridge进程未运行"
fi

echo ""
echo "📋 4. 检查Gazebo里程计话题数据..."
if timeout 5s gz topic -e -t /model/X3/odometry >/tmp/gz_odometry.log 2>&1; then
    echo "✅ 成功读取Gazebo的/model/X3/odometry样本"
else
    echo "❌ 无法从Gazebo读取/model/X3/odometry"
fi

echo ""
echo "📋 5. 检查/X3/odometry话题数据..."
if ros2 topic list | grep -q "/X3/odometry"; then
    echo "✅ /X3/odometry话题存在"
    echo "话题信息："
    ros2 topic info /X3/odometry
    echo ""
    echo "尝试获取一条消息："
    timeout 5s ros2 topic echo /X3/odometry --once || echo "  没有接收到数据"
else
    echo "❌ /X3/odometry话题不存在"
fi

echo ""
echo "📋 8. 检查Gazebo模型状态..."
echo "Gazebo中的模型列表："
timeout 5s gz topic -t /gazebo/get_world_state -e || echo "  无法获取世界状态"

echo ""
echo "📋 9. 检查日志文件..."
echo ""
echo "Gazebo启动日志（最后10行）："
tail -10 /tmp/gazebo_debug.log 2>/dev/null || echo "  日志文件不存在"
echo ""
echo "🔍 调试完成！"
echo "================================================================"
echo "问题诊断总结："
echo "1. 如果Gazebo话题中没有/model/X3/odometry，说明X3模型没有正确加载odometry插件"
echo "2. 如果Gazebo话题存在但ROS2话题不存在，说明桥接配置有问题"
echo "3. 如果ROS2话题存在但没有数据，说明Gazebo仿真没有正确运行"
echo ""
echo "请检查上述输出来确定问题所在。"
echo ""
echo "按Ctrl+C停止所有进程..."

# 保持脚本运行，让用户可以进一步测试
wait
