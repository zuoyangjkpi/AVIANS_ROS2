# AVIANS ROS2 PORT1 - 仿真测试指南

## 概述

这个项目实现了一个完整的多无人机协同感知和跟踪系统，包括：

- **YOLO12人员检测**: 基于最新YOLO12模型的实时人员检测
- **3D投影模型**: 将2D检测结果投影到3D世界坐标
- **分布式卡尔曼滤波**: 多机协同目标跟踪
- **无人机控制**: 智能航点控制和目标跟踪

## 快速开始

### 1. 环境要求

- Ubuntu 22.04/24.04
- ROS2 Humble/Jazzy
- Gazebo Garden/Harmonic
- 已编译的工作空间

### 2. 启动仿真

```bash
# 方法1: 使用完整仿真脚本
./launch_drone_simulation.py

# 方法2: 使用ROS2 launch
ros2 launch drone_state_publisher simulation.launch.py

# 方法3: 使用简单Gazebo仿真
ros2 launch drone_description gz.launch.py
```

### 3. 监控系统状态

```bash
# 检查话题状态
./test_topics.py

# 手动检查关键话题
ros2 topic list
ros2 topic echo /person_detections
ros2 topic echo /machine_1/target_tracker/pose
```

## 系统架构

### 核心组件

1. **Gazebo仿真环境**
   - 3D世界场景
   - 无人机模型 (X3)
   - 人员模型
   - 物理引擎

2. **感知系统**
   - 摄像头传感器: `/camera/image_raw`
   - YOLO12检测器: `/person_detections`
   - 调试图像: `/detection_debug_image`

3. **定位与变换**
   - 无人机位姿: `/machine_1/pose`
   - TF变换树: `world -> machine_1 -> camera`
   - 静态变换发布

4. **跟踪系统**
   - 3D投影: `/machine_1/object_detections/projected_to_world`
   - 卡尔曼滤波: `/machine_1/target_tracker/pose`
   - 不确定性量化

5. **控制系统**
   - 航点控制器
   - 速度命令: `/X3/cmd_vel`
   - 里程计: `/X3/odom`

### 数据流

```
Camera → YOLO12 → 3D Projection → Kalman Filter → Drone Controller
   ↓         ↓           ↓              ↓              ↓
/image_raw /detections /projected   /tracker/pose  /cmd_vel
```

## 关键话题

### 输入话题
- `/camera/image_raw` - 摄像头图像
- `/camera/camera_info` - 摄像头标定信息
- `/machine_1/pose` - 无人机位姿

### 输出话题
- `/person_detections` - 人员检测结果
- `/detection_debug_image` - 检测可视化
- `/machine_1/target_tracker/pose` - 跟踪目标位姿
- `/X3/cmd_vel` - 无人机控制命令

### 调试话题
- `/machine_1/object_detections/projected_to_world` - 3D投影结果
- `/machine_1/target_tracker/twist` - 目标速度
- `/neural_network_feedback` - 检测反馈

## 参数配置

### YOLO12检测器参数
```yaml
confidence_threshold: 0.3    # 置信度阈值
iou_threshold: 0.3          # NMS阈值
desired_class: 0            # 人员类别ID
max_update_rate_hz: 4.0     # 最大更新频率
```

### 卡尔曼滤波参数
```yaml
initialUncertaintyPosXY: 100.0     # 初始位置不确定性
falsePositiveThresholdSigma: 3.0   # 误检阈值
reset_time_threshold: 30.0         # 重置时间阈值
```

### 无人机控制参数
```yaml
max_horizontal_speed: 0.8    # 最大水平速度
max_vertical_speed: 0.5      # 最大垂直速度
waypoint_tolerance: 0.15     # 航点容差
```

## 故障排除

### 常见问题

1. **编译错误**
   ```bash
   # 检查依赖
   rosdep install --from-paths src --ignore-src -r -y
   
   # 重新编译
   colcon build --continue-on-error
   ```

2. **话题无数据**
   ```bash
   # 检查节点状态
   ros2 node list
   ros2 node info /yolo12_detector_node
   
   # 检查话题连接
   ros2 topic info /camera/image_raw
   ```

3. **Gazebo启动失败**
   ```bash
   # 检查环境变量
   echo $GZ_SIM_RESOURCE_PATH
   echo $GAZEBO_MODEL_PATH
   
   # 重置Gazebo
   gz sim --reset
   ```

4. **YOLO模型缺失**
   ```bash
   # 检查模型文件
   ls src/neural_network_detector/third_party/YOLOs-CPP/models/
   
   # 下载YOLO12模型 (如果需要)
   wget -O yolo12n.onnx https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo12n.onnx
   ```

### 性能优化

1. **GPU加速**
   - 设置 `use_gpu: true` (需要CUDA)
   - 安装ONNX Runtime GPU版本

2. **检测频率调整**
   - 降低 `max_update_rate_hz`
   - 增加 `min_detection_interval`

3. **跟踪参数调优**
   - 调整 `falsePositiveThresholdSigma`
   - 优化过程噪声参数

## 扩展功能

### 多机协同
```bash
# 启动多个无人机
ros2 launch drone_state_publisher simulation.launch.py robot_id:=1 num_robots:=3
ros2 launch drone_state_publisher simulation.launch.py robot_id:=2 num_robots:=3
ros2 launch drone_state_publisher simulation.launch.py robot_id:=3 num_robots:=3
```

### 自定义世界
- 修改 `src/drone_description/worlds/drone_world.sdf`
- 添加更多人员模型
- 调整环境参数

### 新的检测类别
- 修改 `desired_class` 参数
- 更新YOLO标签文件
- 调整检测阈值

## 开发指南

### 添加新节点
1. 创建包: `ros2 pkg create --build-type ament_cmake my_package`
2. 添加依赖到 `package.xml` 和 `CMakeLists.txt`
3. 实现节点逻辑
4. 更新launch文件

### 调试技巧
```bash
# 启用调试日志
ros2 run neural_network_detector yolo12_detector_node --ros-args --log-level DEBUG

# 录制数据包
ros2 bag record -a

# 可视化TF树
ros2 run tf2_tools view_frames
```

## 参考资料

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [YOLO Documentation](https://docs.ultralytics.com/)
- [OpenCV Documentation](https://docs.opencv.org/)

---

**注意**: 这是一个研究原型系统，用于学术研究和技术验证。在实际部署前请进行充分测试。

