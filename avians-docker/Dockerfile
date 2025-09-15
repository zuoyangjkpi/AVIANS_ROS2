FROM ubuntu:24.04

# 设置时区避免交互式安装
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# 安装基础工具
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    software-properties-common \
    lsb-release \
    gnupg2 \
    tzdata \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 创建非root用户
RUN useradd -m -s /bin/bash aviansuser && \
    echo "aviansuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 切换到aviansuser
USER aviansuser
WORKDIR /home/aviansuser

# 安装Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    bash ~/miniconda.sh -b -p ~/miniconda3 && \
    rm ~/miniconda.sh && \
    ~/miniconda3/bin/conda init bash

# 设置PATH包含conda
ENV PATH="/home/aviansuser/miniconda3/bin:$PATH"

# 复制conda环境文件并创建环境
COPY airship_ros2_env.yml /tmp/airship_ros2_env.yml
RUN conda env create -f /tmp/airship_ros2_env.yml

# 激活conda环境
SHELL ["/bin/bash", "-c"]
RUN echo "conda activate airship_ros2" >> ~/.bashrc

# 安装ROS2 Jazzy
RUN sudo add-apt-repository universe -y && \
    sudo apt update && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sudo apt update && \
    sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 安装ROS2依赖包
RUN sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-ament-package \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-image-transport \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-pose-cov-ops \
    ros-jazzy-eigen3-cmake-module

# 安装系统依赖
RUN sudo apt install -y \
    build-essential \
    cmake \
    libcurl4-openssl-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    libeigen3-dev \
    python3-pip \
    python3-dev

# 初始化rosdep
RUN sudo rosdep init && rosdep update

# 设置ROS2环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 克隆AVIANS_ROS2_PORT1仓库
RUN git clone https://github.com/zuoyangjkpi/Edited_PORT1.git AVIANS_ROS2_PORT1

# 切换到工作目录
WORKDIR /home/aviansuser/AVIANS_ROS2_PORT1

# 运行setup脚本
RUN source ~/.bashrc && \
    conda activate airship_ros2 && \
    source /opt/ros/jazzy/setup.bash && \
    ./setup_avians_ros2.sh

# 构建工作空间
RUN source ~/.bashrc && \
    conda activate airship_ros2 && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select uav_msgs neural_network_msgs && \
    colcon build --packages-select ros2_utils && \
    colcon build --packages-select pose_cov_ops_interface projection_model target_tracker_distributed_kf neural_network_detector && \
    colcon build

# 设置工作空间环境
RUN echo "source /home/aviansuser/AVIANS_ROS2_PORT1/install/setup.bash" >> ~/.bashrc

# 创建启动脚本
RUN echo '#!/bin/bash' > /home/aviansuser/start_avians.sh && \
    echo 'source ~/.bashrc' >> /home/aviansuser/start_avians.sh && \
    echo 'conda activate airship_ros2' >> /home/aviansuser/start_avians.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/aviansuser/start_avians.sh && \
    echo 'source /home/aviansuser/AVIANS_ROS2_PORT1/install/setup.bash' >> /home/aviansuser/start_avians.sh && \
    echo 'echo "AVIANS ROS2 环境已就绪!"' >> /home/aviansuser/start_avians.sh && \
    echo 'exec "$@"' >> /home/aviansuser/start_avians.sh && \
    chmod +x /home/aviansuser/start_avians.sh

# 设置入口点
ENTRYPOINT ["/home/aviansuser/start_avians.sh"]
CMD ["/bin/bash"]

# 暴露常用端口
EXPOSE 11311 7400 7401 7402 7403 7404 7405