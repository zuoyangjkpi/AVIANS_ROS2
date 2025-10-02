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

## Pixhawk硬件在环测试 (HITL)

### HITL vs SITL 对比

| 特性 | SITL (Software-in-the-Loop) | HITL (Hardware-in-the-Loop) |
|------|----------------------------|----------------------------|
| **PX4运行位置** | PC上的软件仿真 | 真实Pixhawk硬件 |
| **传感器数据** | Gazebo模拟 | Gazebo模拟 (通过MAVLink注入) |
| **控制器** | PX4固件完全仿真 | 真实PX4固件 |
| **执行器输出** | 发送到Gazebo | 发送到Gazebo (通过MAVLink) |
| **用途** | 算法开发、快速迭代 | 硬件验证、固件测试 |
| **优势** | 无需硬件、调试方便 | 测试真实固件、验证硬件性能 |
| **劣势** | 无法发现硬件问题 | 需要硬件连接、设置复杂 |

**本项目使用HITL的原因**：
- 验证自定义PX4模块 (`avians_ros2_interface`, `avians_controllers`) 在真实硬件上的性能
- 测试uXRCE-DDS在实际硬件上的通信稳定性
- 确保固件烧录和参数配置正确

### HITL硬件连接

#### 1. 物理连接

```
┌─────────────────┐         USB           ┌──────────────────┐
│  PC/Orin NX     │◄─────────────────────►│  Pixhawk 6X      │
│                 │                        │                  │
│  - Gazebo       │      TELEM2 (Serial)   │  - PX4固件       │
│  - QGC          │◄─────────────────────►│  - 传感器        │
│  - XRCE Agent   │      921600 baud       │  - 控制器        │
└─────────────────┘                        └──────────────────┘
         │                                          │
         │                                          │
         ▼                                          ▼
    Gazebo仿真环境                            真实固件运行
    (提供传感器数据)                          (执行控制算法)
```

#### 2. 接线说明

**TELEM2端口连接** (用于uXRCE-DDS通信):
- Pixhawk TELEM2 TX → USB-Serial RX
- Pixhawk TELEM2 RX → USB-Serial TX
- Pixhawk TELEM2 GND → USB-Serial GND

**USB连接** (用于MAVLink/QGroundControl):
- Pixhawk USB-C → PC USB端口

#### 3. 串口设备检测

```bash
# 查找串口设备
ls -l /dev/ttyUSB* /dev/ttyACM*

# 典型输出:
# /dev/ttyUSB0 (FTDI USB-Serial) - 用于TELEM2
# /dev/ttyACM0 (Pixhawk USB) - 用于MAVLink
```

### HITL测试流程

#### 步骤1: 配置PX4为HITL模式

1. **使用QGroundControl连接Pixhawk**
```bash
# 确保QGC可以通过USB连接
# USB设备通常是 /dev/ttyACM0
```

2. **设置HITL参数**
```bash
# 在QGC的MAVLink控制台或参数编辑器中:
param set SYS_HITL 1              # 启用HITL模式
param set MAV_TYPE 13             # 设置为六轴飞行器
param set SYS_AUTOSTART 4001      # 通用六轴飞行器配置
param save
```

3. **配置uXRCE-DDS (TELEM2)**
```bash
param set UXRCE_DDS_CFG 102       # TELEM2端口
param set SER_TEL2_BAUD 921600    # 波特率
param save
```

4. **重启Pixhawk**
```bash
reboot
```

#### 步骤2: 启动Gazebo仿真

```bash
# 启动Gazebo (提供传感器数据)
cd ~/AVIANS_ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch drone_description drone_world.launch.py
```

#### 步骤3: 连接QGroundControl到仿真

1. 启动QGroundControl
2. QGC会自动通过USB连接到Pixhawk
3. 在QGC中应该能看到仿真中的飞机状态

#### 步骤4: 启动uXRCE-DDS Agent

```bash
# 方法1: 自动检测串口
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

# 方法2: 手动指定
# 如果自动检测失败，检查实际设备名称
ls -l /dev/ttyUSB* /dev/ttyACM*
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

验证连接:
```bash
# 检查PX4话题是否出现
ros2 topic list | grep /fmu
# 应该看到 /fmu/in/* 和 /fmu/out/* 话题
```

#### 步骤5: 启动AVIANS系统

**方法1: 使用一键测试脚本**
```bash
cd ~/AVIANS_ROS2
chmod +x pixhawk_hitl_test.sh
./pixhawk_hitl_test.sh

# 选择选项:
# 1 - 检查PX4连接状态
# 2 - 启动Micro-XRCE-DDS Agent
# 3 - 完整HITL集成测试
# 4 - 监控PX4话题
# 5 - 停止所有进程
```

**方法2: 手动启动各节点**
```bash
# 终端1: YOLO检测器
ros2 run neural_network_detector yolo_detector_node

# 终端2: NMPC控制器
ros2 run drone_nmpc_tracker nmpc_node

# 终端3: PX4桥接
ros2 run px4_bridge px4_bridge_node

# 终端4: 状态发布器
ros2 run drone_state_publisher state_publisher_node

# 终端5: 投影模型
ros2 run projection_model projection_node

# 其他必要节点...
```

#### 步骤6: 激活Offboard模式并解锁

1. **通过QGroundControl**:
   - 切换飞行模式到 "Offboard"
   - 点击 "Armed" 解锁飞机

2. **或通过ROS2发送命令**:
```bash
# 发送解锁命令
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
"{command: 400, param1: 1.0}"

# 发送Offboard模式命令
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
"{command: 176, param1: 6.0}"
```

#### 步骤7: 启用跟踪

```bash
# 发布跟踪使能命令
ros2 topic pub /drone/tracking_enabled std_msgs/msg/Bool "data: true"
```

### HITL调试技巧

#### 1. 监控关键话题

```bash
# Pixhawk状态
ros2 topic echo /fmu/out/vehicle_status

# 控制指令 (ROS2→PX4)
ros2 topic echo /fmu/in/trajectory_setpoint

# 位置反馈 (PX4→ROS2)
ros2 topic echo /fmu/out/vehicle_local_position

# NMPC输出
ros2 topic echo /drone/control/waypoint_command
```

#### 2. PX4系统日志

```bash
# 通过MAVLink控制台查看日志
# 在QGC的控制台中:
dmesg                          # 系统消息
avians_ros2_interface status   # 自定义模块状态
uxrce_dds_client status        # DDS客户端状态
```

#### 3. 性能分析

```bash
# 检查话题频率
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic hz /drone/control/waypoint_command

# 检查延迟
ros2 topic delay /fmu/out/vehicle_local_position
```

### HITL常见问题

#### 问题1: Pixhawk未进入HITL模式

**症状**: QGC显示正常传感器数据而非仿真数据

**解决方案**:
```bash
# 检查HITL参数
param show SYS_HITL
# 应该显示 SYS_HITL = 1

# 如果为0,重新设置
param set SYS_HITL 1
param save
reboot
```

#### 问题2: uXRCE-DDS连接失败

**症状**: `ros2 topic list` 看不到 `/fmu/*` 话题

**解决方案**:
1. 检查串口权限
```bash
sudo chmod 666 /dev/ttyUSB0
# 或永久添加用户到dialout组
sudo usermod -a -G dialout $USER
# 注销重新登录生效
```

2. 检查PX4的DDS客户端状态
```bash
# 在QGC MAVLink控制台:
uxrce_dds_client status
# 应该显示 "connected"
```

3. 检查波特率匹配
```bash
# PX4侧
param show SER_TEL2_BAUD  # 应该是 921600

# Agent侧
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

#### 问题3: Gazebo和Pixhawk不同步

**症状**: Gazebo中飞机位置与PX4报告位置不一致

**解决方案**:
1. 检查MAVLink连接
2. 验证传感器数据流向
3. 重启Gazebo和Pixhawk

#### 问题4: Offboard模式无法激活

**症状**: 切换到Offboard模式失败

**解决方案**:
1. 确保trajectory_setpoint正在发布
```bash
ros2 topic hz /fmu/in/trajectory_setpoint
# 应该有稳定的频率 (>2Hz)
```

2. 检查Offboard控制模式
```bash
ros2 topic echo /fmu/in/offboard_control_mode
# position字段应该为true
```

### HITL测试检查清单

- [ ] PX4固件已烧录 (`px4_fmu-v6x_avians_v1.px4`)
- [ ] HITL模式已启用 (`SYS_HITL=1`)
- [ ] uXRCE-DDS已配置 (`UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600`)
- [ ] 串口连接正常 (`/dev/ttyUSB0` 可访问)
- [ ] Gazebo仿真正在运行
- [ ] QGroundControl已连接
- [ ] Micro-XRCE-DDS Agent正在运行
- [ ] ROS2话题 `/fmu/*` 可见
- [ ] NMPC控制器正在发布指令
- [ ] PX4桥接节点正在运行
- [ ] Offboard模式已激活
- [ ] 飞机已解锁

## 相关文件

- **ROS2 px4_bridge**: `~/AVIANS_ROS2/src/px4_bridge/src/px4_bridge_node.cpp`
- **PX4模块**: `~/AVIANS_PX4/src/modules/avians_ros2_interface/`
- **PX4控制器**: `~/AVIANS_PX4/src/modules/avians_controllers/`
- **板级配置**: `~/AVIANS_PX4/boards/px4/fmu-v6x/avians_v1.px4board`
- **HITL测试脚本**: `~/AVIANS_ROS2/pixhawk_hitl_test.sh`
- **完整集成测试**: `~/AVIANS_ROS2/comprehensive_test_suite.sh`
