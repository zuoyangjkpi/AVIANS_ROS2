#!/bin/bash

# AVIANS ROS2 Final Test Script - 创建更明显的人物模型进行YOLO测试
# =================================================================

echo "🚀 AVIANS Final Test - Enhanced Human Detection"
echo "================================================"

# 色彩输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# 清理之前的进程
print_status $YELLOW "🧹 清理现有进程..."
pkill -f "gz sim" 2>/dev/null
pkill -f "yolo12_detector_node" 2>/dev/null
pkill -f "nmpc_tracker_node" 2>/dev/null
pkill -f "nmpc_test_node" 2>/dev/null
sleep 2

# 设置环境变量
export DRONE_WS="/home/zuoyangjkpi/AVIANS_ROS2_PORT1"
export PYTHONPATH="/home/zuoyangjkpi/AVIANS_ROS2_PORT1/install/drone_nmpc_tracker/lib/python3.12/site-packages:/home/zuoyangjkpi/AVIANS_ROS2_PORT1/install/neural_network_msgs/lib/python3.12/site-packages"

print_status $YELLOW "📝 创建增强的人物模型..."

# 创建一个更像真实人类的SDF模型文件
cat > /tmp/enhanced_human.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="enhanced_human">
    <static>false</static>
    <pose>0 0 1.0 0 0 0</pose>
    
    <!-- 头部 -->
    <link name="head">
      <pose>0 0 1.7 0 0 0</pose>
      <collision name="head_collision">
        <geometry>
          <sphere><radius>0.12</radius></sphere>
        </geometry>
      </collision>
      <visual name="head_visual">
        <geometry>
          <sphere><radius>0.12</radius></sphere>
        </geometry>
        <material>
          <ambient>0.8 0.7 0.6 1</ambient>
          <diffuse>0.8 0.7 0.6 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.03</ixx><iyy>0.03</iyy><izz>0.03</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- 躯干 -->
    <link name="torso">
      <pose>0 0 1.3 0 0 0</pose>
      <collision name="torso_collision">
        <geometry>
          <cylinder><radius>0.2</radius><length>0.6</length></cylinder>
        </geometry>
      </collision>
      <visual name="torso_visual">
        <geometry>
          <cylinder><radius>0.2</radius><length>0.6</length></cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.0 1.0 1</ambient>
          <diffuse>0.0 0.0 1.0 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>40.0</mass>
        <inertia>
          <ixx>1.5</ixx><iyy>1.5</iyy><izz>0.8</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- 左臂 -->
    <link name="left_arm">
      <pose>-0.3 0 1.4 1.57 0 0</pose>
      <collision name="left_arm_collision">
        <geometry>
          <cylinder><radius>0.06</radius><length>0.5</length></cylinder>
        </geometry>
      </collision>
      <visual name="left_arm_visual">
        <geometry>
          <cylinder><radius>0.06</radius><length>0.5</length></cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.7 0.6 1</ambient>
          <diffuse>0.8 0.7 0.6 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.03</ixx><iyy>0.03</iyy><izz>0.006</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- 右臂 -->
    <link name="right_arm">
      <pose>0.3 0 1.4 1.57 0 0</pose>
      <collision name="right_arm_collision">
        <geometry>
          <cylinder><radius>0.06</radius><length>0.5</length></cylinder>
        </geometry>
      </collision>
      <visual name="right_arm_visual">
        <geometry>
          <cylinder><radius>0.06</radius><length>0.5</length></cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.7 0.6 1</ambient>
          <diffuse>0.8 0.7 0.6 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.03</ixx><iyy>0.03</iyy><izz>0.006</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- 左腿 -->
    <link name="left_leg">
      <pose>-0.1 0 0.5 0 0 0</pose>
      <collision name="left_leg_collision">
        <geometry>
          <cylinder><radius>0.08</radius><length>0.8</length></cylinder>
        </geometry>
      </collision>
      <visual name="left_leg_visual">
        <geometry>
          <cylinder><radius>0.08</radius><length>0.8</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.5</ixx><iyy>0.5</iyy><izz>0.03</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- 右腿 -->
    <link name="right_leg">
      <pose>0.1 0 0.5 0 0 0</pose>
      <collision name="right_leg_collision">
        <geometry>
          <cylinder><radius>0.08</radius><length>0.8</length></cylinder>
        </geometry>
      </collision>
      <visual name="right_leg_visual">
        <geometry>
          <cylinder><radius>0.08</radius><length>0.8</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.5</ixx><iyy>0.5</iyy><izz>0.03</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- 关节连接 -->
    <joint name="head_torso" type="fixed">
      <parent>torso</parent>
      <child>head</child>
    </joint>
    
    <joint name="torso_left_arm" type="fixed">
      <parent>torso</parent>
      <child>left_arm</child>
    </joint>
    
    <joint name="torso_right_arm" type="fixed">
      <parent>torso</parent>
      <child>right_arm</child>
    </joint>
    
    <joint name="torso_left_leg" type="fixed">
      <parent>torso</parent>
      <child>left_leg</child>
    </joint>
    
    <joint name="torso_right_leg" type="fixed">
      <parent>torso</parent>
      <child>right_leg</child>
    </joint>
    
  </model>
</sdf>
EOF

print_status $GREEN "✅ 增强人物模型已创建"

# 创建修改的world文件，包含增强的人物模型
print_status $YELLOW "📝 创建测试世界文件..."

cp src/drone_description/worlds/drone_world.sdf /tmp/test_world.sdf

# 在world文件中添加增强的人物模型
cat >> /tmp/test_world.sdf << 'EOF'

    <!-- 增强的人物模型 -->
    <include>
      <uri>file:///tmp/enhanced_human.sdf</uri>
      <pose>1 1 0 0 0 0</pose>
    </include>
    
    <!-- 第二个人物模型 - 不同位置 -->
    <include>
      <uri>file:///tmp/enhanced_human.sdf</uri>
      <pose>-2 2 0 0 0 0</pose>
      <name>human2</name>
    </include>

  </world>
</sdf>
EOF

print_status $GREEN "✅ 测试世界文件已创建"

print_status $YELLOW "🚀 启动Gazebo仿真..."

# 启动Gazebo
GZ_SIM_RESOURCE_PATH="/tmp:$GZ_SIM_RESOURCE_PATH" gz sim /tmp/test_world.sdf &
GAZEBO_PID=$!

sleep 8

# 检查Gazebo是否运行
if pgrep -f "gz sim" > /dev/null; then
    print_status $GREEN "✅ Gazebo仿真已启动"
else
    print_status $RED "❌ Gazebo启动失败"
    exit 1
fi

print_status $YELLOW "📷 启动相机话题..."

# 启动桥接以发布相机话题
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image &
BRIDGE_PID=$!

sleep 3

print_status $YELLOW "🧠 启动YOLO检测器..."

# 查找YOLO模型文件
MODEL_PATH=$(find . -name "yolo12n.onnx" | head -1)
LABELS_PATH=$(find . -name "coco.names" | head -1)

if [ -z "$MODEL_PATH" ] || [ -z "$LABELS_PATH" ]; then
    print_status $RED "❌ 找不到YOLO模型文件"
    exit 1
fi

print_status $GREEN "📁 使用模型: $MODEL_PATH"
print_status $GREEN "📁 使用标签: $LABELS_PATH"

# 启动YOLO检测器
/opt/ros/jazzy/bin/ros2 run neural_network_detector yolo12_detector_node \
    --ros-args \
    -p "model_path:=$MODEL_PATH" \
    -p "labels_path:=$LABELS_PATH" \
    -p "use_gpu:=false" \
    -p "confidence_threshold:=0.3" \
    -p "desired_class:=0" \
    -p "publish_debug_image:=true" \
    -p "max_update_rate_hz:=2.0" &
YOLO_PID=$!

sleep 5

print_status $YELLOW "📊 启动可视化节点..."

# 启动可视化节点
/usr/bin/python3 visualization_node.py &
VIZ_PID=$!

sleep 3

print_status $GREEN "🎉 系统启动完成！"
print_status $BLUE "==============================="
print_status $YELLOW "💡 系统状态:"

# 检查各个进程
if pgrep -f "gz sim" > /dev/null; then
    print_status $GREEN "  ✅ Gazebo仿真运行中"
else
    print_status $RED "  ❌ Gazebo仿真未运行"
fi

if pgrep -f "yolo12_detector_node" > /dev/null; then
    print_status $GREEN "  ✅ YOLO检测器运行中"
else
    print_status $RED "  ❌ YOLO检测器未运行"
fi

if pgrep -f "visualization_node" > /dev/null; then
    print_status $GREEN "  ✅ 可视化节点运行中"
else
    print_status $RED "  ❌ 可视化节点未运行"
fi

print_status $BLUE "==============================="
print_status $YELLOW "🔍 监控检测结果..."

# 监控检测话题
for i in {1..10}; do
    print_status $YELLOW "检测轮次 $i/10:"
    
    # 检查相机话题
    if timeout 2s ros2 topic echo /camera/image_raw --once > /dev/null 2>&1; then
        print_status $GREEN "  ✅ 相机图像可用"
    else
        print_status $RED "  ❌ 相机图像不可用"
    fi
    
    # 检查检测话题
    DETECTION_MSG=$(timeout 3s ros2 topic echo /person_detections --once 2>/dev/null)
    if [ -n "$DETECTION_MSG" ]; then
        NUM_DETECTIONS=$(echo "$DETECTION_MSG" | grep -c "detection_score")
        if [ "$NUM_DETECTIONS" -gt 0 ]; then
            print_status $GREEN "  ✅ 检测到 $NUM_DETECTIONS 个人"
        else
            print_status $YELLOW "  ⚠️  检测话题活跃但没有检测结果"
        fi
    else
        print_status $RED "  ❌ 没有检测话题数据"
    fi
    
    sleep 2
done

print_status $BLUE "==============================="
print_status $GREEN "🎯 测试完成!"
print_status $YELLOW "💡 使用以下命令查看实时检测:"
print_status $YELLOW "   ros2 topic echo /person_detections"
print_status $YELLOW "   rviz2 (查看可视化)"
print_status $YELLOW "💡 使用Ctrl+C停止所有进程"

# 等待用户中断
wait