#!/bin/bash

# 自动化测试脚本 - 验证无人机跟踪功能
# ================================================

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

# 设置环境
export DRONE_WS="/home/zuoyangjkpi/AVIANS_ROS2_PORT1"
cd "$DRONE_WS"

print_status $BLUE "🚀 自动化测试：无人机跟踪功能"
print_status $BLUE "================================"

# 第一步：启动Gazebo
print_status $YELLOW "步骤1: 启动Gazebo仿真..."
ros2 launch drone_description gz.launch.py > /tmp/gazebo_test.log 2>&1 &
GAZEBO_PID=$!
sleep 10

# 检查Gazebo是否启动成功
if pgrep -f "gz sim" > /dev/null; then
    print_status $GREEN "✅ Gazebo启动成功"
else
    print_status $RED "❌ Gazebo启动失败"
    exit 1
fi

# 第二步：检查话题
print_status $YELLOW "步骤2: 检查关键话题..."
sleep 3

print_status $YELLOW "检查odometry话题..."
if ros2 topic list | grep -q "/X3/odometry"; then
    print_status $GREEN "✅ /X3/odometry 话题存在"
    
    # 检查话题是否有数据
    timeout 5s ros2 topic echo /X3/odometry --once > /tmp/odom_test.log 2>&1
    if [ $? -eq 0 ]; then
        print_status $GREEN "✅ odometry话题正在发布数据"
    else
        print_status $RED "❌ odometry话题无数据"
    fi
else
    print_status $RED "❌ /X3/odometry 话题不存在"
fi

print_status $YELLOW "检查camera话题..."
if ros2 topic list | grep -q "/camera/image_raw"; then
    print_status $GREEN "✅ /camera/image_raw 话题存在"
else
    print_status $RED "❌ /camera/image_raw 话题不存在"
fi

# 第三步：启动NMPC跟踪器
print_status $YELLOW "步骤3: 启动NMPC跟踪器..."
python3 src/drone_nmpc_tracker/scripts/nmpc_tracker_node > /tmp/nmpc_tracker_test.log 2>&1 &
NMPC_PID=$!
sleep 5

# 第四步：启动人物模拟器
print_status $YELLOW "步骤4: 启动人物模拟器..."
python3 src/drone_nmpc_tracker/scripts/nmpc_test_node > /tmp/nmpc_test_node.log 2>&1 &
TEST_PID=$!
sleep 3

# 第五步：启用控制
print_status $YELLOW "步骤5: 启用无人机控制..."
ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
ENABLE_PID=$!
sleep 2

# 第六步：检查控制指令
print_status $YELLOW "步骤6: 检查控制指令输出..."
sleep 3

if ros2 topic list | grep -q "/X3/cmd_vel"; then
    print_status $GREEN "✅ /X3/cmd_vel 话题存在"
    
    # 检查是否有控制指令
    timeout 10s ros2 topic echo /X3/cmd_vel --once > /tmp/cmd_vel_test.log 2>&1
    if [ $? -eq 0 ]; then
        print_status $GREEN "✅ 控制指令正在发布!"
        
        # 显示控制指令内容
        print_status $BLUE "控制指令内容:"
        cat /tmp/cmd_vel_test.log
        
        print_status $GREEN "🎉 测试成功：无人机控制系统工作正常！"
    else
        print_status $RED "❌ 没有控制指令输出"
        
        print_status $YELLOW "检查NMPC日志..."
        tail -20 /tmp/nmpc_tracker_test.log
    fi
else
    print_status $RED "❌ /X3/cmd_vel 话题不存在"
fi

# 第七步：性能监控
print_status $YELLOW "步骤7: 监控系统性能（10秒）..."
for i in {1..10}; do
    echo -n "."
    sleep 1
done
echo

# 检查话题频率
print_status $BLUE "话题频率检查:"
echo "Odometry频率:"
timeout 3s ros2 topic hz /X3/odometry 2>/dev/null || echo "  无数据"

echo "控制指令频率:"
timeout 3s ros2 topic hz /X3/cmd_vel 2>/dev/null || echo "  无数据"

# 清理
print_status $YELLOW "清理进程..."
kill $GAZEBO_PID $NMPC_PID $TEST_PID $ENABLE_PID 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "nmpc" 2>/dev/null
pkill -f "ros2 topic pub" 2>/dev/null

print_status $BLUE "================================"
print_status $BLUE "测试完成！检查上方结果。"
print_status $YELLOW "日志文件位置:"
print_status $YELLOW "  - Gazebo: /tmp/gazebo_test.log"
print_status $YELLOW "  - NMPC跟踪器: /tmp/nmpc_tracker_test.log"
print_status $YELLOW "  - 人物模拟器: /tmp/nmpc_test_node.log"