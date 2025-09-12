#!/bin/bash

# 修复后的系统验证测试
# =======================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

export DRONE_WS="/home/zuoyangjkpi/AVIANS_ROS2_PORT1"
cd "$DRONE_WS"

print_status $BLUE "🎯 测试修复后的无人机系统"
print_status $BLUE "============================"

# 停止所有现有进程
print_status $YELLOW "停止现有进程..."
pkill -f gz 2>/dev/null
pkill -f nmpc 2>/dev/null  
pkill -f ros2 2>/dev/null
sleep 3

# 步骤1：启动Gazebo
print_status $YELLOW "步骤1: 启动Gazebo..."
ros2 launch drone_description gz.launch.py > /tmp/test_gazebo.log 2>&1 &
sleep 15

# 检查Gazebo
if pgrep -f "gz sim" > /dev/null; then
    print_status $GREEN "✅ Gazebo启动成功"
else
    print_status $RED "❌ Gazebo启动失败"
    exit 1
fi

# 步骤2：检查话题
print_status $YELLOW "步骤2: 检查关键话题..."
if ros2 topic list | grep -q "/X3/odometry"; then
    print_status $GREEN "✅ odometry话题存在"
else
    print_status $RED "❌ odometry话题缺失"
fi

# 步骤3：启动NMPC跟踪器
print_status $YELLOW "步骤3: 启动修复后的NMPC跟踪器..."
python3 src/drone_nmpc_tracker/scripts/nmpc_tracker_node > /tmp/test_nmpc.log 2>&1 &
sleep 5

# 步骤4：启用控制
print_status $YELLOW "步骤4: 启用无人机控制..."
ros2 topic pub -r 1 /X3/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
sleep 3

# 步骤5：检查无人机运动
print_status $YELLOW "步骤5: 监控无人机运动（20秒）..."

# 记录初始位置
initial_pos=$(ros2 topic echo /X3/odometry --once | grep -A3 "position:" | grep "z:" | awk '{print $2}')
print_status $BLUE "初始高度: $initial_pos"

# 等待运动
sleep 20

# 记录最终位置  
final_pos=$(ros2 topic echo /X3/odometry --once | grep -A3 "position:" | grep "z:" | awk '{print $2}')
print_status $BLUE "最终高度: $final_pos"

# 计算高度变化
height_change=$(echo "$final_pos - $initial_pos" | bc -l)
height_change_abs=$(echo "$height_change" | sed 's/-//')

if (( $(echo "$height_change_abs > 0.5" | bc -l) )); then
    print_status $GREEN "🎉 无人机正在运动！高度变化: ${height_change}m"
    print_status $GREEN "✅ 修复成功！"
else
    print_status $YELLOW "⚠️  高度变化较小: ${height_change}m"
fi

# 步骤6：检查NMPC日志
print_status $YELLOW "步骤6: NMPC控制器状态..."
if grep -q "Search mode" /tmp/test_nmpc.log; then
    print_status $GREEN "✅ 搜索模式激活"
else
    print_status $YELLOW "⚠️  未见搜索模式日志"
fi

# 显示最近的控制指令
print_status $YELLOW "最近的控制指令:"
timeout 3s ros2 topic echo /X3/cmd_vel --once 2>/dev/null || print_status $RED "无控制指令"

# 显示关键日志
print_status $BLUE "NMPC日志摘录:"
tail -5 /tmp/test_nmpc.log

print_status $BLUE "============================"  
print_status $GREEN "测试完成！"

# 可选：保持运行让用户观察
read -p "按Enter键停止系统..." dummy

# 清理
pkill -f gz
pkill -f nmpc
pkill -f ros2