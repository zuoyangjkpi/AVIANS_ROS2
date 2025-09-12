#!/bin/bash

# 自动运行comprehensive_test_suite.sh的选项5
echo "5" | source /home/zuoyangjkpi/miniconda3/etc/profile.d/conda.sh && conda activate airship_ros2 && export DRONE_WS="/home/zuoyangjkpi/AVIANS_ROS2_PORT1" && cd "$DRONE_WS" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ./comprehensive_test_suite.sh