#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for NMPC drone tracking with Gazebo visualization
    """
    
    # Get package directories
    drone_description = get_package_share_directory("drone_description")
    
    # Paths
    models_path = os.path.join(drone_description, "models")
    world_file = os.path.join(drone_description, "worlds", "drone_world.sdf")
    config_file = os.path.join(drone_description, "config", "bridge.yaml")
    
    # Environment variables for Gazebo
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
    
    # Graphics settings for better compatibility
    libgl_env = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
    gallium_env = SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe")
    
    # Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", f"-v 4 -r {world_file}")
        ]
    )
    
    # ROS-Gazebo bridge for communication
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{'config_file': config_file}],
        remappings=[
            ('/model/drone/cmd_vel', '/X3/cmd_vel'),
        ]
    )

    odometry_bridge = Node(
        package="drone_state_publisher",
        executable="gazebo_odometry_bridge",
        name="gazebo_odometry_bridge",
        output="screen",
        parameters=[
            {"input_topic": "/X3/odometry_raw"},
            {"output_topic": "/X3/odometry"},
            {"child_frame": "X3/base_link"},
        ]
    )

    # Static transforms
    world_to_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        output="screen"
    )

    # NMPC tracker node
    nmpc_tracker = Node(
        package="drone_nmpc_tracker",
        executable="nmpc_tracker_node",
        name="nmpc_tracker_node",
        output="screen",
        parameters=[
            {"drone_frame": "X3/base_link"},
            {"world_frame": "world"},
            {"camera_frame": "camera_link"},
            {"control_frequency": 10.0},
            {"person_timeout": 2.0},
        ]
    )

    # Simulated person detector (instead of test_node)
    person_simulator = Node(
        package="drone_nmpc_tracker",
        executable="nmpc_test_node",
        name="person_simulator",
        output="screen"
    )

    # Enable NMPC controller
    enable_publisher = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "-r", "1", "/nmpc/enable", "std_msgs/msg/Bool", "data: true"
        ],
        output="screen"
    )
    
    return LaunchDescription([
        # Environment setup
        libgl_env,
        gallium_env,
        gazebo_resource_path,
        set_gazebo_model_path,
        
        # Launch Gazebo
        gazebo,
        odometry_bridge,
        
        # Start bridge after a delay
        TimerAction(
            period=5.0,
            actions=[gz_ros2_bridge]
        ),
        
        # Start transforms and nodes after Gazebo is ready
        TimerAction(
            period=8.0,
            actions=[
                world_to_map_tf,
                person_simulator,
                nmpc_tracker,
            ]
        ),
        
        # Enable tracking after everything is ready
        TimerAction(
            period=12.0,
            actions=[enable_publisher]
        ),
    ])
