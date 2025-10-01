# PX4 Bridge Package

Bridge between AVIANS ROS2 system and PX4 autopilot using uXRCE-DDS.

## Overview

This package enables communication between the AVIANS ROS2 control system (running on NVIDIA Orin NX) and the Pixhawk 6X autopilot running PX4 v1.16.0 firmware.

## Architecture

```
AVIANS ROS2 (Orin NX)                    Pixhawk 6X (PX4 v1.16.0)
━━━━━━━━━━━━━━━━━━━━                    ━━━━━━━━━━━━━━━━━━━━━━━━

┌─────────────────────┐                 ┌──────────────────────┐
│  NMPC Controller    │                 │   uXRCE-DDS Agent    │
│  /drone/control/*   │                 │   /fmu/in/*          │
└──────────┬──────────┘                 └──────────┬───────────┘
           │                                       │
           │                                       │
           v                                       v
┌─────────────────────┐  uXRCE-DDS    ┌──────────────────────┐
│  PX4 Bridge Node    │◄─────────────►│  PX4 Firmware        │
│  /fmu/in/*          │   (Serial/    │  AVIANS Controller   │
│  /fmu/out/*         │    Ethernet)  │  uORB Topics         │
└─────────────────────┘                └──────────────────────┘
```

## Communication Topics

### From AVIANS to PX4 (`/fmu/in/`):
- `/fmu/in/offboard_control_mode` - Control mode selection
- `/fmu/in/trajectory_setpoint` - Position/velocity/acceleration commands
- `/fmu/in/vehicle_command` - Arm/disarm and mode commands

### From PX4 to AVIANS (`/fmu/out/`):
- `/fmu/out/vehicle_status` - PX4 system status
- `/fmu/out/vehicle_odometry` - Vehicle state estimation

### AVIANS Internal Topics:
- `/X3/odometry` - Drone odometry from sensors
- `/drone/control/velocity_setpoint` - NMPC velocity commands
- `/drone/control/waypoint_command` - Waypoint commands
- `/drone/state` - State machine status
- `/drone/controller/status` - NMPC status

## Setup

### 1. Install px4_msgs

```bash
cd ~/AVIANS_ROS2_PORT1/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.16
cd ~/AVIANS_ROS2_PORT1
colcon build --packages-select px4_msgs
```

### 2. Build px4_bridge

```bash
cd ~/AVIANS_ROS2_PORT1
colcon build --packages-select px4_bridge
source install/setup.bash
```

### 3. Configure PX4 uXRCE-DDS

On Pixhawk 6X, enable uXRCE-DDS client:

```bash
# Via MAVLink console or QGroundControl parameters:
UXRCE_DDS_CFG = 1  # Enable on TELEM2 (default)
# Or = 102 for Ethernet
```

### 4. Start uXRCE-DDS Agent on Orin NX

```bash
# Install MicroXRCEAgent
sudo apt install ros-jazzy-micro-xrce-dds-agent

# Start agent (choose one):
# For serial (TELEM2):
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600

# For Ethernet:
micro-ros-agent udp4 --port 8888
```

### 5. Launch Bridge

```bash
ros2 launch px4_bridge px4_bridge.launch.py
```

## Testing

### Check PX4 connection:
```bash
ros2 topic list | grep /fmu
```

You should see topics like:
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/out/vehicle_status`

### Monitor bridge status:
```bash
ros2 topic echo /fmu/out/vehicle_status
```

## Integration with AVIANS

The bridge automatically:
1. Subscribes to NMPC control commands
2. Converts ROS2 messages to PX4 format
3. Handles coordinate frame conversions (ENU ↔ NED)
4. Publishes PX4 status back to ROS2

## Frame Conventions

- **ROS2/AVIANS**: ENU (East-North-Up)
- **PX4**: NED (North-East-Down)

The bridge handles conversion automatically:
```cpp
// ROS2 (ENU) -> PX4 (NED)
px4_north = ros2_x
px4_east  = -ros2_y  // Negated!
px4_down  = -ros4_z  // Negated!
```

## Next Steps

See the main documentation for:
- PX4 firmware module development
- Custom controller integration
- Airframe configuration
