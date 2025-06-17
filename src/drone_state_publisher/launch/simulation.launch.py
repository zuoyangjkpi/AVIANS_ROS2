#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import conditions
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    drone_description = get_package_share_directory("drone_description")
    neural_network_detector_dir = get_package_share_directory("neural_network_detector")
    
    # File paths
    models_path = os.path.join(drone_description, "models")
    world_file = os.path.join(drone_description, "worlds", "drone_world.sdf")
    config_file = os.path.join(drone_description, "config", "bridge.yaml")
    rviz_config = os.path.join(drone_description, "config", "drone.rviz")
    
    # YOLO model paths
    default_yolo_model_path = os.path.join(
        neural_network_detector_dir, "YOLOs-CPP", "models", "yolo12n.onnx"
    )
    default_yolo_labels_path = os.path.join(
        neural_network_detector_dir, "YOLOs-CPP", "models", "coco.names"
    )

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time for all nodes'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id', default_value='1',
        description='Robot ID for this instance'
    )
    
    declare_num_robots = DeclareLaunchArgument(
        'num_robots', default_value='1',
        description='Total number of robots'
    )

    # Environment variables for Gazebo
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

    # =============================================================================
    # 1. GAZEBO SIMULATION
    # =============================================================================
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", f"-v 4 -r {world_file}")]
    )

    # ROS-Gazebo bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # =============================================================================
    # 2. DRONE STATE PUBLISHER (NEW - CRITICAL FOR PIPELINE)
    # =============================================================================
    
    drone_state_publisher_node = Node(
        package='drone_state_publisher',
        executable='drone_state_publisher_node',
        name='drone_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'optimal_distance': 4.0,      # 4m from target
            'optimal_height_offset': 7.0, # 2m above target  
            'optimal_angle_offset': 0.0,  # 0° = directly behind target
        }]
    )

    # =============================================================================
    # 3. TF TRANSFORM PUBLISHER  
    # =============================================================================
    
    tf_from_uav_pose_node = Node(
        package='tf_from_uav_pose_ros2',
        executable='tf_from_uav_pose_node',
        name='tf_from_uav_pose',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # Topic mapping - CORRECTED PARAMETER NAMES FROM CPP
            'pose_topic_name': '/machine_1/pose',
            'raw_pose_topic_name': '/machine_1/pose/raw', 
            'std_pose_topic_name': '/machine_1/pose/corr/std',
            'std_raw_pose_topic_name': '/machine_1/pose/raww/std',
            'throttled_pose_topic_name': '/machine_1/throttledPose',
            'throttled_uav_pose_topic_name': '/machine_1/throttledUAVPose',
            
            # Frame IDs - CORRECTED PARAMETER NAMES FROM CPP
            'machine_frame_id': 'machine_1',
            'world_frame_id': 'world', 
            'world_enu_frame_id': 'world_ENU',
            'world_nwu_frame_id': 'world_NWU',
            'camera_frame_id': 'xtion_depth_frame',
            'camera_rgb_optical_frame_id': 'xtion_depth_optical_frame',
            
            # Offset and covariance parameters
            'offset_x': 0.0,
            'offset_y': 0.0,
            'offset_z': 0.0,
            'covariance_x': 0.0,
            'covariance_y': 0.0,
            'covariance_z': 0.0,
            'throttle_rate': 10.0,
            'dont_publish_tfs': False,
            
            # Camera static transform - CORRECTED PARAMETER NAMES FROM CPP
            'camera_static_publish.publish': True,
            'camera_static_publish.tf_parameters': [
                -0.2, 0.0, 0.0,           # Position: 0.2m forward
                -0.173665, 0.000008, 0.984805, 0.000046  # Quaternion from SDF
            ],
            'camera_static_publish.topic': '/machine_1/camera/pose',
            'camera_static_publish.pose_optical_topic': '/machine_1/camera/pose_optical'
        }]
    )

    # =============================================================================
    # 4. NEURAL NETWORK DETECTION
    # =============================================================================
    
    yolo_detector_node = Node(
        package="neural_network_detector",
        executable="yolo12_detector_node", 
        name="yolo12_detector_node",
        output='screen',
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            'model_path': default_yolo_model_path,
            'labels_path': default_yolo_labels_path,
            'use_gpu': False,
            'confidence_threshold': 0.5,
            'iou_threshold': 0.3,
            'desired_class': 0,  # Person class
            'desired_width': 300,
            'desired_height': 300,
            'aspect_ratio': 1.333333333,
            'border_dropoff': 0.05,
            'publish_debug_image': True,
            'max_update_force': True,
            'max_update_rate_hz': 4.0,
            'feedback_timeout_sec': 5.0,
            # Variance parameters
            'var_const_x_min': 0.00387,
            'var_const_x_max': 0.00347,
            'var_const_y_min': 0.00144,
            'var_const_y_max': 0.00452,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),              # FROM: Gazebo camera
            ('detections', '/person_detections'),            # TO: Projection model
            ('detection_count', '/person_detection_count'),  
            ('feedback', '/neural_network_feedback'),        # FROM: Projection model
            ('debug_image', '/detection_debug_image'),
        ]
    )

    # =============================================================================
    # 5. 3D PROJECTION MODEL
    # =============================================================================
    
    projector_node = Node(
        package='projection_model',
        executable='projection_model_node',
        name='model_distance_from_height_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # Topics
            'projected_object_topic': "/machine_1/object_detections/projected_to_world",
            'camera_debug_topic': "/machine_1/object_detections/camera_debug",
            'detections_topic': "/person_detections",
            'tracker_topic': "/machine_1/target_tracker/pose",
            'offset_topic': "/machine_1/target_tracker/offset",
            'feedback_topic': "/neural_network_feedback",
            
            # UPDATED: Use SDF-defined frames directly
            'topics.robot': "/machine_1/pose/raww/std",
            'topics.camera': "/machine_1/camera/pose",
            # REMOVED: optical topic - projection model will use direct TF lookups to X3/camera_optical_frame
            
            # Model parameters
            'height_model_mean': 1.8,
            'height_model_var': 1.0,
            'uncertainty_scale_head': 1.0,
            'uncertainty_scale_feet': 1.0,
            
            # Camera info
            'camera.info_topic': "/camera/camera_info"
        }],
        output='screen'
    )


    # =============================================================================
    # 6. DISTRIBUTED KALMAN FILTER TRACKER
    # =============================================================================
    
    distributed_kf_node = Node(
        package='target_tracker_distributed_kf',
        executable='distributed_kf_node',
        name='distributed_kf_3d',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robotID': LaunchConfiguration('robot_id'),
            'numRobots': LaunchConfiguration('num_robots'),
            
            # Initial uncertainty parameters
            'initialUncertaintyPosXY': 100.0,
            'initialUncertaintyPosZ': 10.0,
            'initialUncertaintyVelXY': 1.0,
            'initialUncertaintyVelZ': 0.5,
            'initialUncertaintyOffsetXY': 1.0,
            'initialUncertaintyOffsetZ': 3.0,
            
            # Process noise parameters
            'noisePosXVar': 0.0,
            'noiseVelXVar': 0.5,
            'noiseOffXVar': 0.02,
            'noisePosYVar': 0.0,
            'noiseVelYVar': 0.5,
            'noiseOffYVar': 0.02,
            'noisePosZVar': 0.0,
            'noiseVelZVar': 0.5,
            'noiseOffZVar': 0.02,
            
            # Bias parameters
            'posGlobalOffsetBiasX': 0.0,
            'posGlobalOffsetBiasY': 0.0,
            'posGlobalOffsetBiasZ': 0.0,
            
            # Decay parameters
            'velocityDecayTime': 3.0,
            'offsetDecayTime': 30.0,
            'falsePositiveThresholdSigma': 6.0,
            
            # Topics
            'pub_topic': '/machine_1/target_tracker/pose',     # TO: Projection model & Drone state
            'velPub_topic': '/machine_1/target_tracker/twist', # TO: Drone state
            'offset_topic': '/machine_1/target_tracker/offset', # TO: Projection model & Drone state
            'pose_topic': '/machine_1/pose',                   # FROM: Drone state publisher
            'measurement_topic_suffix_self': '/machine_1/object_detections/projected_to_world', # FROM: Projection model
            'measurement_topic_suffix': 'object_detections/projected_to_world',
            
            # Filter parameters
            'reset_time_threshold': 10.0,
            'cache_size': 40,
        }],
        output='screen'
    )

    # =============================================================================
    # 7. DRONE WAYPOINT CONTROLLER
    # =============================================================================
    
    waypoint_controller = Node(
        package='drone_description',
        executable='waypoint_controller',
        name='waypoint_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'max_horizontal_speed': 1.0,
            'max_vertical_speed': 0.5,
            'max_yaw_rate': 0.5,
            'waypoint_tolerance': 0.15,
            'yaw_p_gain': 0.3,
            'yaw_d_gain': 0.1,
            'prediction_time': 0.5,
            'enable_debug': True,
            'pure_tracking_mode': False,
            'cmd_vel_topic': '/X3/cmd_vel',     # TO: Gazebo via bridge
            'odom_topic': '/X3/odom',           # FROM: Gazebo via bridge
        }],
        remappings=[
            ('/target_waypoint', '/target_waypoint'),  # FROM: Drone state publisher
        ]
    )

    # =============================================================================
    # 8. STATIC TRANSFORMS - CONNECT GAZEBO TO ROS FRAMES
    # =============================================================================
    
    # Connect Gazebo camera frame to ROS TF tree
    gazebo_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",  # No translation offset
            "0", "0", "0",  # No rotation offset  
            "xtion_depth_optical_frame",  # Parent (ROS frame)
            "X3/X3/base_link/camera_front"  # Child (Gazebo frame)
        ],
        output="screen"
    )
    
    # Original static transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # =============================================================================
    # LAUNCH SEQUENCE WITH PROPER TIMING
    # =============================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_robot_id,
        declare_num_robots,
        
        # Environment
        libgl_env,
        gallium_env,
        gazebo_resource_path,
        
        # 1. Start Gazebo simulation first
        gazebo,
        gz_ros2_bridge,
        
        # 2. Start TF and drone state (foundation nodes)
        TimerAction(period=3.0, actions=[
            tf_from_uav_pose_node,
            drone_state_publisher_node
        ]),
        
        # 3. Start detection pipeline
        TimerAction(period=5.0, actions=[
            yolo_detector_node
        ]),
        
        # 4. Start projection and tracking
        TimerAction(period=7.0, actions=[
            projector_node
        ]),
        
        TimerAction(period=9.0, actions=[
            distributed_kf_node
        ]),
        
        # 5. Start drone control
        TimerAction(period=11.0, actions=[
            waypoint_controller
        ]),
        
        # 6. Start visualization and TF connections
        TimerAction(period=13.0, actions=[
            gazebo_camera_tf,  # CRITICAL: Connect Gazebo camera to ROS TF tree
            static_tf,
            rviz_node
        ])
    ])