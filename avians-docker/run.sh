#!/bin/bash

echo "=========================================="
echo "启动AVIANS ROS2 Docker容器"
echo "=========================================="

# 确保X11转发权限
xhost +local:docker

# 创建shared目录
mkdir -p ./shared

# 启动容器
docker-compose up -d

echo "容器已启动！"
echo ""
echo "使用以下命令进入容器："
echo "docker exec -it avians-ros2-container /bin/bash"
echo ""
echo "或者使用："
echo "./shell.sh"
echo "=========================================="