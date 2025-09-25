#!/bin/bash

set -euo pipefail

echo "=== TF坐标变换诊断脚本 ==="
echo ""

# 确保ROS环境已加载
if [ -f "install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source install/setup.bash
    echo "✅ 加载ROS工作区环境"
else
    echo "❌ 工作区环境文件未找到"
    exit 1
fi

# 常用帧名定义
WORLD_FRAME="world"
BASE_FRAME="X3/camera_link"
OPT_FRAME="X3/camera_rgb_optical_frame"
FRONT_FRAME="X3/camera_link/camera_front"
FRONT_OPT_FRAME="X3/camera_link/camera_front_optical_frame"

echo ""
echo "1. 📋 检查TF树结构..."
VIEW_FRAMES_PDF="/tmp/view_frames.pdf"
VIEW_FRAMES_LOG="/tmp/view_frames.log"
if timeout 5s ros2 run tf2_tools view_frames >"${VIEW_FRAMES_PDF}" 2>"${VIEW_FRAMES_LOG}"; then
    echo "   ✅ 已生成 TF 结构图 (${VIEW_FRAMES_PDF})"
else
    echo "   ⚠️ view_frames 执行失败，可查看 ${VIEW_FRAMES_LOG}"
fi

echo ""
echo "2. 📍 检查相机相关的TF变换..."
if ros2 tf list > /tmp/tf_list.txt 2>/tmp/tf_list.err; then
    echo "   ▸ 当前 TF 列表中的相机帧："
    if ! grep -E "${BASE_FRAME}|${OPT_FRAME}|${FRONT_FRAME}|${FRONT_OPT_FRAME}" /tmp/tf_list.txt; then
        echo "     ⚠️ 未在列表中直接找到相机帧，可能由动态 TF 发布"
    fi
else
    echo "   ⚠️ ros2 tf list 执行失败，详见 /tmp/tf_list.err"
fi

echo "   ▸ /tf_static 中与 camera 相关的条目："
if ! timeout 3s ros2 topic echo --once /tf_static | grep -E "${BASE_FRAME}|${OPT_FRAME}|${FRONT_FRAME}"; then
    echo "     ℹ️ /tf_static 中未出现相机帧，可能全部由动态 TF 提供"
fi

echo ""
echo "3. 🎯 检查具体的相机变换参数..."
for frame in "${OPT_FRAME}" "${FRONT_OPT_FRAME}" "${BASE_FRAME}" "${FRONT_FRAME}"; do
    echo "   ▸ ${WORLD_FRAME} -> ${frame}"
    log_file="/tmp/tf_echo_${frame//\//_}.log"
    if timeout 3s ros2 run tf2_ros tf2_echo "${WORLD_FRAME}" "${frame}" >"${log_file}" 2>&1; then
        sed -n '1,6p' "${log_file}"
    else
        echo "     ⚠️ 无法获取 ${frame} 的 TF，详见 ${log_file}"
    fi
    echo ""
done
echo "   ℹ️ 完整输出已保存至 /tmp/tf_echo_*"

echo ""
echo "4. 🔧 检查 tf_from_uav_pose 配置及 drone_tf 发布情况..."
if [ -f "src/tf_from_uav_pose/config/tf_from_uav_pose_params.yaml" ]; then
    echo "   ▸ tf_from_uav_pose 参数片段："
    grep -A5 -B2 "TFParameters" src/tf_from_uav_pose/config/tf_from_uav_pose_params.yaml
else
    echo "   ⚠️ 未找到 tf_from_uav_pose 参数文件"
fi

if ls /tmp/drone_tf.log >/dev/null 2>&1; then
    echo "   ▸ drone_tf_publisher 日志：/tmp/drone_tf.log (若运行中，可 tail 查看)"
else
    echo "   ℹ️ 未发现 /tmp/drone_tf.log，可确认 python TF 发布器是否已启动"
fi

echo ""
echo "5. ⚙️  检查关键位姿话题..."
echo "   ▸ /machine_1/pose (uav_msgs/UAVPose)"
if ! timeout 3s ros2 topic echo --once /machine_1/pose | head -20; then
    echo "     ⚠️ 暂未发布 /machine_1/pose"
fi

echo ""
echo "   ▸ /X3/odometry (nav_msgs/Odometry)"
if ! timeout 3s ros2 topic echo --once /X3/odometry | head -20; then
    echo "     ⚠️ 暂未发布 /X3/odometry"
fi

echo ""
echo "6. 📐 预期的30°向下倾斜配置验证..."
echo "   期望相机位姿：位置[0.2, 0.0, 0.0]，四元数[0, sin(15°)≈0.2588, 0, cos(15°)≈0.9659]"
echo "   若 TF 输出与此差异较大，请检查 tf_from_uav_pose 与 drone_tf_publisher 的配置"

echo ""
echo "=== 诊断完成 ==="
