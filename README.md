# AVIANS ROS2 PORT1: Autonomous Multi-UAV Perception and Tracking System

![ROS2](https://img.shields.io/badge/ROS-ROS2%20Jazzy-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-orange)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen)
![License](https://img.shields.io/badge/License-Apache%202.0-yellow)

A comprehensive ROS2 migration of the AVIANS (Autonomous Vision-based Intelligent Aerial Navigation System) for multi-UAV perception-driven formation control and target tracking. This system integrates advanced YOLO-based object detection, distributed Kalman filtering, and sophisticated 3D projection models for autonomous aerial surveillance and tracking missions.

## 🚀 Key Features

- **🤖 Advanced Object Detection**: YOLO v12 neural network integration with ONNX Runtime
- **🎯 Distributed Target Tracking**: Multi-UAV collaborative Kalman filtering with 9-dimensional state estimation
- **📐 3D Projection Models**: Accurate 2D-to-3D target localization using statistical height models
- **🌐 Multi-Agent Coordination**: Distributed architecture supporting multiple UAV formations
- **⚡ Real-time Performance**: Optimized for 4Hz control frequency with low-latency processing
- **🔧 ROS2 Native**: Full ROS2 Jazzy integration with modern middleware capabilities

## 📋 System Requirements

### Hardware Requirements
- **CPU**: Multi-core processor (Intel i5 or AMD Ryzen 5 equivalent or better)
- **RAM**: Minimum 8GB, recommended 16GB
- **GPU**: NVIDIA GPU with CUDA support (recommended for optimal performance)
- **Storage**: At least 10GB free space

### Software Requirements
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS**: ROS2 Jazzy Jalopy
- **Python**: Python 3.12+
- **CMake**: Version 3.22+

## 🛠️ Quick Start

### Automated Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/zuoyangjkpi/Edited_PORT1.git
   cd Edited_PORT1
   ```

2. **Run the automated setup script**:
   ```bash
   chmod +x setup_avians_ros2.sh
   ./setup_avians_ros2.sh
   ```

3. **Verify installation**:
   ```bash
   ./test_avians.sh
   ```

### Manual Installation

If you prefer manual installation or encounter issues with the automated script:

1. **Install ROS2 Jazzy**:
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-jazzy-desktop ros-dev-tools
   ```

2. **Install system dependencies**:
   ```bash
   sudo apt install -y \
     python3-colcon-common-extensions \
     python3-rosdep \
     libcurl4-openssl-dev \
     libopencv-dev \
     ros-jazzy-cv-bridge \
     ros-jazzy-image-geometry \
     ros-jazzy-pose-cov-ops
   ```

3. **Install Python dependencies**:
   ```bash
   pip3 install ultralytics onnxruntime opencv-python numpy
   ```

4. **Build the workspace**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

## 🧪 Testing and Validation

### System Test
Run the comprehensive system test to verify all components:
```bash
./test_avians.sh
```

### Drone Detection Simulation
Execute a complete drone person detection simulation:
```bash
python3 test_drone_detection.py
```

This test simulates:
- 🛫 **Takeoff Phase**: Autonomous climb to 5m altitude
- ✈️ **Cruise Phase**: Circular flight pattern for area coverage
- 🔍 **Detection Phase**: Hover and scan for human targets
- 🛬 **Landing Phase**: Controlled descent and landing

## 📦 Package Architecture

### Core Packages

| Package | Description | Status |
|---------|-------------|--------|
| `neural_network_detector` | YOLO v12 object detection with ONNX Runtime | ✅ Active |
| `target_tracker_distributed_kf` | Distributed Kalman filtering for multi-UAV tracking | ✅ Active |
| `projection_model` | 2D-to-3D target localization with statistical models | ✅ Active |
| `pose_cov_ops_interface` | Pose covariance operations for uncertainty propagation | ✅ Active |

### Message Packages

| Package | Description |
|---------|-------------|
| `neural_network_msgs` | Object detection message definitions |
| `uav_msgs` | UAV-specific message types and structures |

### Utility Packages

| Package | Description |
|---------|-------------|
| `ros2_utils` | Common utilities and helper functions |
| `tf_from_uav_pose` | Transform broadcasting for UAV poses |

## 🔧 Configuration

### Neural Network Configuration
Edit `src/neural_network_detector/config/yolo.yaml`:
```yaml
yolo12_detector_node:
  ros__parameters:
    model_path: "path/to/yolo12n.onnx"
    confidence_threshold: 0.5
    iou_threshold: 0.45
    use_gpu: true
```

### Target Tracking Configuration
Modify tracking parameters in the distributed Kalman filter node configuration files.

### Multi-UAV Setup
Configure multiple UAV instances by adjusting the machine namespace parameters in launch files.

## 🚁 Usage Examples

### Single UAV Detection
```bash
# Terminal 1: Start the neural network detector
ros2 run neural_network_detector yolo12_detector_node

# Terminal 2: Start the target tracker
ros2 run target_tracker_distributed_kf distributed_kf_node

# Terminal 3: Publish test images or connect camera
ros2 topic pub /machine_1/xtion/rgb/image_raw sensor_msgs/msg/Image [...]
```

### Multi-UAV Formation
```bash
# Launch multi-UAV system with formation control
ros2 launch [launch_file] num_uavs:=3
```

## 🔬 Technical Specifications

### Detection Performance
- **Model**: YOLO v12n (nano version for real-time performance)
- **Input Resolution**: 640×480 pixels
- **Detection Classes**: 80 COCO classes (person detection optimized)
- **Inference Speed**: ~30 FPS on modern GPUs

### Tracking Accuracy
- **State Estimation**: 9-dimensional state vector (position, velocity, sensor offsets)
- **Update Rate**: 4Hz for control loop integration
- **Tracking Range**: Up to 50m with standard cameras
- **Multi-target Support**: Simultaneous tracking of multiple persons

### Communication
- **Middleware**: ROS2 DDS with reliable QoS profiles
- **Latency**: <100ms end-to-end detection-to-control
- **Bandwidth**: Optimized message sizes for multi-UAV scenarios

## 🛠️ Development and Customization

### Adding New Detection Classes
1. Retrain YOLO model with custom dataset
2. Update class labels in configuration files
3. Modify detection filtering logic if needed

### Extending Tracking Capabilities
1. Modify state vector dimensions in `DistributedKF3D.cpp`
2. Update motion models and observation matrices
3. Adjust covariance parameters for new scenarios

### Multi-UAV Scaling
1. Configure namespace parameters for additional UAVs
2. Update communication topics and message routing
3. Adjust formation control parameters

## 🐛 Troubleshooting

### Common Issues

**Build Errors with CURL**:
```bash
# Add curl linking in CMakeLists.txt
target_link_libraries(your_target curl)
```

**OpenCV Version Warnings**:
- These are compatibility warnings and don't affect functionality
- System uses OpenCV 4.10 while ROS2 expects 4.06

**ONNX Runtime Not Found**:
```bash
# Ensure ONNX Runtime is properly downloaded
cd src/neural_network_detector/third_party/YOLOs-CPP/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
tar -xzf onnxruntime-linux-x64-1.20.1.tgz
```

## 📚 Documentation

- **API Documentation**: Generated with Doxygen (run `doxygen` in project root)
- **Message Interfaces**: See `msg/` directories in respective packages
- **Configuration Files**: Located in `config/` directories

## 🤝 Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Follow ROS2 coding standards
- Add unit tests for new functionality
- Update documentation for API changes
- Ensure backward compatibility when possible

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Original AVIANS system developers
- ROS2 community for excellent middleware
- Ultralytics team for YOLO implementations
- ONNX Runtime team for inference optimization

## 📞 Support

For questions, issues, or contributions:

- **Issues**: [GitHub Issues](https://github.com/zuoyangjkpi/Edited_PORT1/issues)
- **Discussions**: [GitHub Discussions](https://github.com/zuoyangjkpi/Edited_PORT1/discussions)
- **Email**: [Contact Information]

---

**Built with ❤️ for autonomous aerial systems**
