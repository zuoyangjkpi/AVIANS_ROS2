#!/bin/bash

# 逐步调试脚本
# ==============

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

print_status $BLUE "🔧 逐步调试无人机系统"
print_status $BLUE "======================"

# 停止所有进程
print_status $YELLOW "清理所有进程..."
pkill -f gz 2>/dev/null || true
pkill -f nmpc 2>/dev/null || true
pkill -f ros2 2>/dev/null || true
pkill -f rviz 2>/dev/null || true
sleep 3

print_status $GREEN "✅ 进程清理完成"

# 步骤1：测试单独启动RViz
print_status $YELLOW "步骤1: 测试RViz单独启动..."
timeout 5s rviz2 > /tmp/debug_rviz.log 2>&1 &
RVIZ_PID=$!
sleep 2

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    print_status $GREEN "✅ RViz可以单独启动"
    kill $RVIZ_PID 2>/dev/null
else
    print_status $RED "❌ RViz单独启动失败"
    print_status $YELLOW "RViz错误日志:"
    cat /tmp/debug_rviz.log
fi

# 步骤2：最简单的Gazebo启动测试
print_status $YELLOW "步骤2: 启动Gazebo..."
ros2 launch drone_description gz.launch.py > /tmp/debug_gazebo.log 2>&1 &
GAZEBO_PID=$!
sleep 15

if pgrep -f "gz sim" > /dev/null; then
    print_status $GREEN "✅ Gazebo启动成功"
    
    # 检查RViz是否也启动了
    if pgrep -f "rviz" > /dev/null; then
        print_status $GREEN "✅ RViz通过launch文件启动成功"
    else
        print_status $RED "❌ RViz未通过launch文件启动"
    fi
else
    print_status $RED "❌ Gazebo启动失败"
    print_status $YELLOW "Gazebo错误日志:"
    tail -20 /tmp/debug_gazebo.log
    exit 1
fi

# 步骤3：检查话题
print_status $YELLOW "步骤3: 检查关键话题..."
sleep 5

if ros2 topic list | grep -q "/X3/odometry"; then
    print_status $GREEN "✅ odometry话题存在"
    
    # 检查odometry数据
    print_status $YELLOW "检查odometry数据..."
    if timeout 5s ros2 topic echo /X3/odometry --once > /dev/null 2>&1; then
        print_status $GREEN "✅ odometry有数据"
        
        # 显示当前位置
        pos_data=$(ros2 topic echo /X3/odometry --once | grep -A3 "position:")
        print_status $BLUE "当前位置: $pos_data"
    else
        print_status $RED "❌ odometry无数据"
    fi
else
    print_status $RED "❌ odometry话题不存在"
fi

# 步骤4：测试手动控制
print_status $YELLOW "步骤4: 测试手动控制..."

# 启用无人机
print_status $YELLOW "启用无人机..."
ros2 topic pub -1 /X3/enable std_msgs/msg/Bool "data: true"
sleep 2

# 记录初始位置
initial_z=$(ros2 topic echo /X3/odometry --once | grep "z:" | head -1 | awk '{print $2}')
print_status $BLUE "初始Z位置: $initial_z"

# 发送上升指令
print_status $YELLOW "发送上升指令..."
ros2 topic pub -r 5 /X3/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 &
CMD_PID=$!
sleep 10

# 停止指令并检查结果
kill $CMD_PID 2>/dev/null
final_z=$(ros2 topic echo /X3/odometry --once | grep "z:" | head -1 | awk '{print $2}')
print_status $BLUE "最终Z位置: $final_z"

# 计算变化
height_change=$(echo "$final_z - $initial_z" | bc -l)
if (( $(echo "$height_change > 0.3" | bc -l) )); then
    print_status $GREEN "🎉 无人机响应手动控制！高度变化: ${height_change}m"
else
    print_status $RED "❌ 无人机不响应控制，高度变化: ${height_change}m"
fi

print_status $BLUE "======================"
print_status $YELLOW "调试完成。系统将保持运行以供观察..."
print_status $YELLOW "按Ctrl+C停止系统"

# 等待用户中断
wait