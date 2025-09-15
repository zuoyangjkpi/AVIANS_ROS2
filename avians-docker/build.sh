#!/bin/bash

echo "=========================================="
echo "构建AVIANS ROS2 Docker镜像"
echo "=========================================="

# 确保X11转发权限
xhost +local:docker

# 构建Docker镜像
docker-compose build --no-cache

echo "=========================================="
echo "镜像构建完成！"
echo "使用 ./run.sh 启动容器"
echo "=========================================="