# AVIANS ROS2 Docker环境

这个Docker环境包含了完整的AVIANS ROS2 PORT1项目环境，包括：

- Ubuntu 24.04基础镜像
- ROS2 Jazzy完整桌面版
- Conda环境（airship_ros2）
- 所有项目依赖和包
- 自动构建的ROS2工作空间

## 快速开始

### 1. 构建Docker镜像

```bash
cd ~/avians-docker
./build.sh
```

### 2. 启动容器

```bash
./run.sh
```

### 3. 进入容器

```bash
./shell.sh
```

### 4. 停止容器

```bash
./stop.sh
```

## 目录结构

```
~/avians-docker/
├── Dockerfile              # Docker镜像定义
├── docker-compose.yml      # Docker Compose配置
├── airship_ros2_env.yml     # Conda环境配置
├── build.sh                # 构建脚本
├── run.sh                  # 运行脚本
├── shell.sh                # 进入容器脚本
├── stop.sh                 # 停止脚本
├── shared/                 # 容器与宿主机共享目录
└── README.md               # 说明文档
```

## 容器环境说明

- **用户**: aviansuser（非root用户）
- **工作目录**: `/home/aviansuser/AVIANS_ROS2_PORT1`
- **Conda环境**: airship_ros2（自动激活）
- **ROS2版本**: Jazzy
- **网络模式**: host（与宿主机共享网络）

## 环境变量

容器中已自动设置以下环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/aviansuser/AVIANS_ROS2_PORT1/install/setup.bash
conda activate airship_ros2
```

## 使用说明

### 进入容器后可直接使用：

```bash
# 查看可用的ROS2包
ros2 pkg list | grep -E "(neural_network|uav_msgs|pose_cov|projection|target_tracker)"

# 运行测试
cd /home/aviansuser/AVIANS_ROS2_PORT1
./test_avians.sh

# 启动节点示例
ros2 run neural_network_detector yolo12_detector_node
```

### 开发模式

`shared/` 目录可用于在宿主机和容器间共享文件，适合开发调试。

### 数据持久化

工作空间和conda环境使用Docker卷进行持久化存储，容器删除后数据不会丢失。

## 故障排除

### GUI应用显示问题

如果遇到图形界面无法显示，确保运行了：

```bash
xhost +local:docker
```

### 权限问题

容器内用户为aviansuser，拥有sudo权限，密码为空。

### 网络问题

容器使用host网络模式，与宿主机共享网络。如有冲突可修改docker-compose.yml。

## 自定义配置

可根据需要修改：

- `Dockerfile`: 调整基础环境和依赖
- `docker-compose.yml`: 修改端口、卷挂载等配置
- `airship_ros2_env.yml`: 更新conda环境

## 在新电脑上使用

1. 复制整个 `avians-docker` 文件夹到新电脑
2. 确保新电脑已安装Docker和Docker Compose
3. 运行 `./build.sh` 构建镜像
4. 运行 `./run.sh` 启动容器
5. 运行 `./shell.sh` 进入容器开始使用

项目就可以在新电脑上正常运行了！