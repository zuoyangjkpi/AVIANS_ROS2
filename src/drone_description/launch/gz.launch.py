
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    # Paths
    drone_description = get_package_share_directory("drone_description")
    models_path = os.path.join(drone_description, "models")
    world_file = os.path.join(drone_description, "worlds", "drone_world.sdf")
    config_file = os.path.join(drone_description, "config", "bridge.yaml")
    rviz_config = os.path.join(drone_description, "config", "drone.rviz")
    urdf_file = os.path.join(drone_description, "urdf", "x3_drone.urdf")

    # Environment variables
    libgl_env = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
    gallium_env = SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            str(Path(drone_description).parent.resolve()),
            models_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        ])
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=os.path.join(drone_description, "lib")
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", f"-v 4 -r {world_file}")
        ]
    )

    # ROS-Gazebo bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # Robot state publisher
    robot_description = Command(['cat ', urdf_file])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}]
    )

    # Static transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )

    # Drone enabler - Use Python script instead of ros2 topic pub
    # This creates a simple publisher that continuously enables the drone
    
    # Waypoint controller
    controller = Node(
        package='drone_description',
        executable='waypoint_controller',
        name='waypoint_controller',
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    yolo_node = Node(
        package="neural_network_detector",
        executable="yolo12_detector_node",
        name="yolo12_detector_node",
        output="screen",
        parameters=[
            {'model_path': '/home/zuoyangjkpi/AVIANS_ROS2_PORT1/src/neural_network_detector/third_party/YOLOs-CPP/models/yolo12n.onnx'},
            {'labels_path': '/home/zuoyangjkpi/AVIANS_ROS2_PORT1/src/neural_network_detector/third_party/YOLOs-CPP/models/coco.names'},
            {'use_gpu': False},
            {'confidence_threshold': 0.5},
            {'desired_class': 0},  # person class (COCO class 0)
            {'max_update_rate_hz': 1.0}
        ]
    )

    return LaunchDescription([
        libgl_env,
        gallium_env,
        gazebo_resource_path,
        set_gazebo_model_path,
        yolo_node,
        gz_plugin_path,
        gazebo,
        gz_ros2_bridge,
        robot_state_publisher,
        TimerAction(
            period=10.0,
            actions=[ 
                      static_tf,
                      controller,
                      rviz_node
                    ]
        )
       
    ])
