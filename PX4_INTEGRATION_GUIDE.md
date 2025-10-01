# AVIANS PX4硬件集成指南

## 概述

本指南说明如何将AVIANS ROS2跟踪系统与PX4 Pixhawk 6X硬件集成。

## 架构设计

```
┌────────────────────────────────────────────────────────────┐
│  ROS2 (NVIDIA Orin NX)                                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  YOLO检测 → 人员位置                                  │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  NMPC控制器 → 位置设定点 (ENU坐标系)                 │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  px4_bridge                                          │  │
│  │  - 订阅: /drone/control/waypoint_command             │  │
│  │  - 订阅: /drone/control/attitude_command             │  │
│  │  - 转换: ENU → NED                                   │  │
│  │  - 发布: /fmu/in/trajectory_setpoint                 │  │
│  │  - 发布: /fmu/in/offboard_control_mode               │  │
│  └───────────────────────┬──────────────────────────────┘  │
└────────────────────────────┼───────────────────────────────┘
                             │ uXRCE-DDS (UDP/Serial)
                             ▼
┌────────────────────────────────────────────────────────────┐
│  PX4固件 (Pixhawk 6X)                                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  avians_ros2_interface (新增模块)                    │  │
│  │  - 接收trajectory_setpoint                           │  │
│  │  - 安全限制检查                                       │  │
│  │  - 超时保护 (500ms)                                  │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼ uORB                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  mc_pos_control (PX4原生位置控制器)                  │  │
│  │  - 位置/速度PID控制                                  │  │
│  │  - 输出姿态设定点                                    │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  mc_att_control (PX4原生姿态控制器)                  │  │
│  │  - 姿态PID控制                                       │  │
│  │  - 输出角速率设定点                                  │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  mc_rate_control (PX4原生角速率控制器)               │  │
│  │  - 角速率PID控制                                     │  │
│  │  - 输出力矩设定点                                    │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  control_allocator (PX4原生控制分配)                 │  │
│  │  - 力矩 → 8个电机推力                               │  │
│  │  - 考虑电机布局和旋转方向                           │  │
│  └───────────────────────┬──────────────────────────────┘  │
│                          ▼                                  │
│           [电机1] [电机2] ... [电机8]                       │
└────────────────────────────────────────────────────────────┘
```

## 第一步：安装依赖

### 1.1 编译工作区内置的 px4_msgs（ROS2侧）

`px4_msgs` 包已经随仓库提供在 `src/custom_msgs/px4_msgs` 路径下，无需单独克隆，只需在工作区内编译：

```bash
cd ~/AVIANS_ROS2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs
```

### 1.2 安装ARM交叉编译工具链（PX4侧）

```bash
sudo apt update
sudo apt install -y gcc-arm-none-eabi
```

### 1.3 安装kconfiglib

```bash
pip3 install kconfiglib
# 或者用conda
# ~/miniconda3/bin/pip install kconfiglib
```

## 第二步：编译ROS2 px4_bridge节点

```bash
cd ~/AVIANS_ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # 如果之前编译过其他包
colcon build --packages-select px4_bridge --symlink-install
```

## 第三步：编译PX4固件

```bash
cd ~/AVIANS_PX4
make px4_fmu-v6x_avians
```

这将编译包含`avians_ros2_interface`模块的PX4固件。

## 第四步：烧录固件到Pixhawk 6X

```bash
cd ~/AVIANS_PX4
make px4_fmu-v6x_avians upload
```

或者使用QGroundControl烧录生成的固件文件：
```
~/AVIANS_PX4/build/px4_fmu-v6x_avians/px4_fmu-v6x_avians.px4
```

## 第五步：配置PX4参数

使用QGroundControl或MAVLink控制台配置以下参数：

### 5.1 uXRCE-DDS客户端配置

```bash
# 使用TELEM2端口 (Serial 4)
param set UXRCE_DDS_CFG 102

# 设置波特率 (921600)
param set SER_TEL2_BAUD 921600

# 保存参数
param save
```

### 5.2 AVIANS模块参数

```bash
# 最大水平速度 (m/s)
param set AVIANS_MAX_VEL_XY 4.5

# 最大垂直速度 (m/s)
param set AVIANS_MAX_VEL_Z 2.0

# 最大偏航率 (rad/s)
param set AVIANS_MAX_YAW_RATE 1.5

# 保存参数
param save
```

### 5.3 Offboard模式配置

```bash
# RC失控异常类型 (0=Position, 4=Terminate)
param set COM_RCL_EXCEPT 4

# Offboard失控超时 (秒)
param set COM_OF_LOSS_T 1.0

# 保存参数
param save
```

## 第六步：启动ROS2系统

### 6.1 在Orin NX上启动uXRCE-DDS Agent

```bash
# UDP方式 (通过网络连接)
MicroXRCEAgent udp4 -p 8888

# 或者Serial方式 (通过UART连接)
# MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### 6.2 启动AVIANS ROS2节点

```bash
cd ~/AVIANS_ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 启动检测节点
ros2 run neural_network_detector yolo_detector_node &

# 启动NMPC控制器
ros2 run drone_nmpc_tracker nmpc_node &

# 启动PX4桥接节点
ros2 run px4_bridge px4_bridge_node &
```

### 6.3 启动Gazebo仿真（可选，用于测试）

```bash
ros2 launch drone_description drone_world.launch.py
```

## 第七步：验证通信

### 7.1 检查ROS2话题

```bash
# 查看所有话题
ros2 topic list

# 应该看到PX4话题：
# /fmu/in/offboard_control_mode
# /fmu/in/trajectory_setpoint
# /fmu/in/vehicle_command
# /fmu/out/vehicle_status
# /fmu/out/vehicle_local_position
```

### 7.2 检查消息流

```bash
# 查看NMPC输出
ros2 topic echo /drone/control/waypoint_command

# 查看发送给PX4的指令
ros2 topic echo /fmu/in/trajectory_setpoint

# 查看PX4状态
ros2 topic echo /fmu/out/vehicle_status
```

### 7.3 检查PX4 MAVLink控制台

```bash
# 进入PX4 NSH控制台
# 检查avians_ros2_interface状态
avians_ros2_interface status

# 应该显示：
# Running
# Trajectory valid: yes
# Last trajectory update: XX ms ago
```

## 坐标系转换

### ROS2 (ENU) → PX4 (NED)

px4_bridge自动处理坐标转换：

| ROS2 (ENU) | PX4 (NED) | 转换公式 |
|------------|-----------|---------|
| X (East)   | Y (East)  | NED_Y = ENU_X |
| Y (North)  | X (North) | NED_X = ENU_Y |
| Z (Up)     | Z (Down)  | NED_Z = -ENU_Z |
| Yaw (ENU)  | Yaw (NED) | NED_yaw = -ENU_yaw + π/2 |

## 消息流说明

### NMPC → px4_bridge

**话题**: `/drone/control/waypoint_command`
**类型**: `geometry_msgs/PoseStamped`
**内容**:
```
position:
  x: 目标东向位置 (m, ENU)
  y: 目标北向位置 (m, ENU)
  z: 目标高度 (m, ENU)
```

**话题**: `/drone/control/attitude_command`
**类型**: `geometry_msgs/Vector3Stamped`
**内容**:
```
vector:
  x: roll (rad, 通常为0)
  y: pitch (rad, 通常为0)
  z: yaw (rad, ENU)
```

### px4_bridge → PX4

**话题**: `/fmu/in/trajectory_setpoint`
**类型**: `px4_msgs/TrajectorySetpoint`
**内容**:
```
position[0]: 北向位置 (m, NED)
position[1]: 东向位置 (m, NED)
position[2]: 向下位置 (m, NED)
yaw: 偏航角 (rad, NED)
```

**话题**: `/fmu/in/offboard_control_mode`
**类型**: `px4_msgs/OffboardControlMode`
**内容**:
```
position: true
velocity: false
acceleration: false
attitude: false
body_rate: false
```

## 故障排除

### 问题1: px4_bridge无法连接到PX4

**症状**: `ros2 topic list`看不到`/fmu/*`话题

**解决方案**:
1. 检查uXRCE-DDS Agent是否运行
```bash
ps aux | grep MicroXRCEAgent
```

2. 检查PX4的uXRCE-DDS客户端状态
```bash
# 在PX4 MAVLink控制台
uxrce_dds_client status
```

3. 检查网络/串口连接
```bash
# UDP方式ping测试
ping <pixhawk_ip>

# Serial方式检查设备
ls -l /dev/ttyUSB*
```

### 问题2: PX4未响应ROS2指令

**症状**: trajectory_setpoint有数据，但飞机不动

**解决方案**:
1. 检查Offboard模式是否激活
```bash
# QGroundControl: Flight Modes → Offboard
```

2. 检查avians_ros2_interface模块状态
```bash
avians_ros2_interface status
# 应该显示 "Trajectory valid: yes"
```

3. 检查PX4是否解锁
```bash
commander status
# 应该显示 "Arming state: armed"
```

### 问题3: 坐标不匹配

**症状**: 飞机向错误方向飞行

**解决方案**:
1. 检查EKF2原点设置
```bash
param show EKF2_EV_POS_X
param show EKF2_EV_POS_Y
param show EKF2_EV_POS_Z
```

2. 验证坐标转换
```bash
# 监控原始和转换后的值
ros2 topic echo /drone/control/waypoint_command &
ros2 topic echo /fmu/in/trajectory_setpoint
```

### 问题4: 控制指令超时

**症状**: avians_ros2_interface报告 "Trajectory setpoint timeout"

**解决方案**:
1. 检查NMPC是否正常运行
```bash
ros2 node list | grep nmpc
ros2 topic hz /drone/control/waypoint_command
# 应该显示 ~10-50 Hz
```

2. 检查px4_bridge是否运行
```bash
ros2 node list | grep px4_bridge
```

3. 增加网络带宽或降低发布频率

## 性能调优

### PX4参数调优

```bash
# 位置控制器增益
param set MPC_XY_P 1.0
param set MPC_Z_P 1.0

# 速度控制器增益
param set MPC_XY_VEL_P 0.15
param set MPC_XY_VEL_I 0.02
param set MPC_Z_VEL_P 0.3
param set MPC_Z_VEL_I 0.05

# 最大速度和加速度
param set MPC_XY_VEL_MAX 4.5
param set MPC_Z_VEL_MAX_DN 2.0
param set MPC_Z_VEL_MAX_UP 2.0
param set MPC_ACC_HOR 3.0
```

### ROS2优化

1. 调整QoS设置（在px4_bridge_node.cpp中）
2. 使用实时内核（PREEMPT_RT）
3. 设置CPU亲和性

## 安全注意事项

1. **首次飞行前**:
   - 在仿真中充分测试
   - 设置RC遥控器为备份控制
   - 配置failsafe参数
   - 进行地面测试

2. **飞行中**:
   - 保持RC遥控器在手
   - 监控电池电压
   - 注意Offboard超时
   - 准备紧急降落

3. **紧急情况**:
   - RC切换到手动模式立即接管
   - 使用Kill Switch切断电机
   - QGroundControl紧急停止

## 相关文件

- ROS2 px4_bridge: `~/AVIANS_ROS2/src/px4_bridge/src/px4_bridge_node.cpp`
- PX4模块: `~/AVIANS_PX4/src/modules/avians_ros2_interface/`
- 板级配置: `~/AVIANS_PX4/boards/px4/fmu-v6x/avians.px4board`
- 模块README: `~/AVIANS_PX4/src/modules/avians_ros2_interface/README.md`
