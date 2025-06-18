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
from launch.substitutions import LaunchConfiguration
from launch import conditions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    neural_network_detector_dir = get_package_share_directory("neural_network_detector")
    
    # Construct absolute paths to model files
    default_yolo_model_path = os.path.join(
        neural_network_detector_dir, 
        "YOLOs-CPP",
        "models", 
        "yolo12n.onnx"
    )
    
    default_yolo_labels_path = os.path.join(
        neural_network_detector_dir, 
        "YOLOs-CPP",
        "models", 
        "coco.names"
    )

    # Launch arguments
    declare_use_person_tracking = DeclareLaunchArgument(
        'use_person_tracking',
        default_value='true',
        description='Enable person tracking functionality'
    )
    
    declare_yolo_model_path = DeclareLaunchArgument(
        'yolo_model_path',
        default_value=default_yolo_model_path,
        description='Path to YOLO12 model file'
    )
    
    declare_yolo_labels_path = DeclareLaunchArgument(
        'yolo_labels_path',
        default_value=default_yolo_labels_path,
        description='Path to YOLO12 labels file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for all nodes'
    )

    # TF from UAV Pose (ROS2) - MOST IMPORTANT
   

    # # Debug checker - runs first to verify bridge topics
    # debug_check = ExecuteProcess(
    #     cmd=['bash', '-c', '''
    #     echo "Waiting for bridge topics to be available..."
    #     timeout=30
    #     count=0
        
    #     while [ $count -lt $timeout ]; do
    #         if ros2 topic list | grep -q "/machine_1/pose/raww/std" && \
    #            ros2 topic list | grep -q "/machine_1/camera/pose" && \
    #            ros2 topic list | grep -q "/machine_1/camera/pose_optical" && \
    #            ros2 topic list | grep -q "/clock"; then
    #             echo "✓ All required bridge topics are available!"
    #             break
    #         fi
    #         echo "Waiting for bridge topics... ($count/$timeout)"
    #         sleep 1
    #         ((count++))
    #     done
        
    #     if [ $count -ge $timeout ]; then
    #         echo "⚠️ Timeout waiting for bridge topics. Check your ROS1-ROS2 bridge!"
    #         echo "Missing topics:"
    #         ros2 topic list | grep -q "/machine_1/pose/raww/std" || echo "  - /machine_1/pose/raww/std"
    #         ros2 topic list | grep -q "/machine_1/camera/pose" || echo "  - /machine_1/camera/pose"  
    #         ros2 topic list | grep -q "/machine_1/camera/pose_optical" || echo "  - /machine_1/camera/pose_optical"
    #         ros2 topic list | grep -q "/clock" || echo "  - /clock"
    #     fi
    #     '''],
    #     output='screen'
    # )

    # RViz2
    

    def get_base_params():
        return {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
          parameters=[{
            **get_base_params()
          }],
        output="screen"
    )

    tf_from_uav_pose_node = Node(
        package='tf_from_uav_pose',
        executable='tf_from_uav_pose_node',
        name='tf_from_uav_pose',
        parameters=[
            # # Load ALL parameters from YAML file
            # LaunchConfiguration('config_file'),
            # Only override essential launch-time parameters
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # YOLO12 Detector Node - starts early since it's independent
    yolo_detector_node = Node(
        package="neural_network_detector",
        executable="yolo12_detector_node", 
        name="yolo12_detector_node",
        output='screen',
        parameters=[{
            **get_base_params(),
            'model_path': default_yolo_model_path,
            'labels_path': default_yolo_labels_path,
            'use_gpu': False,
            'confidence_threshold': 0.3,  # LOWERED for more detections
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
            'var_const_x_min': 0.00387,
            'var_const_x_max': 0.00347,
            'var_const_y_min': 0.00144,
            'var_const_y_max': 0.00452,
        }],
        remappings=[
            ('image_raw', '/firefly_1/xtion/rgb/image_raw'),
            ('detections', '/person_detections'),
            ('detection_count', '/person_detection_count'),  
            ('feedback', '/neural_network_feedback'),
            ('debug_image', '/detection_debug_image'),
        ]
    )
    # Create the projector node - waits for bridge topics
    projector_node = Node(
        package='projection_model',
        executable='projection_model_node',
        name='model_distance_from_height_node',
        parameters=[{
            **get_base_params(),
            'projected_object_topic': "/machine_1/object_detections/projected_to_world",
            'camera_debug_topic': "/machine_1/object_detections/camera_debug",
            'detections_topic': "/person_detections",
            'tracker_topic': "/machine_1/target_tracker/pose",
            'offset_topic': "/machine_1/target_tracker/offset",
            'feedback_topic': "/neural_network_feedback",
            
            'topics.robot': "/machine_1/pose/raww/std",
            'topics.camera': "/machine_1/camera/pose",
            'topics.optical': "/machine_1/camera/pose_optical",
            
            'height_model_mean': 1.8,
            'height_model_var': 1.0,
            'uncertainty_scale_head': 1.0,
            'uncertainty_scale_feet': 1.0,
            
            'camera.info_topic': "/machine_1/video/camera_info"
        }],
        output='screen',
        # arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Distributed Kalman Filter Node - starts last
    distributed_kf_node = Node(
        package='target_tracker_distributed_kf',
        executable='distributed_kf_node',
        name='distributed_kf_3d',
        parameters=[{
            'use_sim_time': True,
            'robotID': 1,
            'numRobots': 1,
            
            # Initial uncertainty parameters
            'initialUncertaintyPosXY': 100.0,
            'initialUncertaintyPosZ': 10.0,
            'initialUncertaintyVelXY': 1.0,
            'initialUncertaintyVelZ': 0.5,
            'initialUncertaintyOffsetXY': 1.0,
            'initialUncertaintyOffsetZ': 3.0,
            
            # Noise parameters
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
            
            # False positive threshold
            'falsePositiveThresholdSigma': 10.0,
            
            # Topics
            'pub_topic': '/machine_1/target_tracker/pose',
            'velPub_topic': '/machine_1/target_tracker/twist',
            'offset_topic': '/machine_1/target_tracker/offset',
            'pose_topic': '/machine_1/pose',
            'measurement_topic_suffix_self': '/machine_1/object_detections/projected_to_world',
            'measurement_topic_suffix': 'object_detections/projected_to_world',
            
            # Other parameters
            'reset_time_threshold': 10.0,
            'cache_size': 40,
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_person_tracking,
        declare_yolo_model_path,
        declare_yolo_labels_path,
        declare_use_sim_time,
        
        #declare_use_sim_time,
        # tf_from_uav_pose_node,
        # TF node starts immediately
        tf_from_uav_pose_node,
        # distributed_kf_node,
        projector_node,
        yolo_detector_node,
        # Other nodes with delays
        # TimerAction(period=2.0, actions=[yolo_detector_node]),
        # TimerAction(period=5.0, actions=[projector_node]),
        # TimerAction(period=8.0, actions=[distributed_kf_node]),
                 
        rviz_node
    ])