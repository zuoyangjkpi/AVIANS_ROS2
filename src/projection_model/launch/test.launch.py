#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    neural_network_detector_dir = get_package_share_directory("neural_network_detector")

    # Define default model paths using substitution
    default_yolo_model_path = PathJoinSubstitution([
        neural_network_detector_dir,
        "YOLOs-CPP",
        "models",
        "yolo12n.onnx"
    ])
    
    default_yolo_labels_path = PathJoinSubstitution([
        neural_network_detector_dir,
        "YOLOs-CPP",
        "models",
        "coco.names"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robot instances to launch'
        ),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=default_yolo_model_path,
            description='Path to YOLO model file'
        ),
        DeclareLaunchArgument(
            'yolo_labels_path',
            default_value=default_yolo_labels_path,
            description='Path to YOLO labels file'
        ),
        OpaqueFunction(function=launch_robot_nodes)
    ])

def launch_robot_nodes(context, *args, **kwargs):
    num_robots = int(context.launch_configurations['num_robots'])
    yolo_model_path = context.launch_configurations['yolo_model_path']
    yolo_labels_path = context.launch_configurations['yolo_labels_path']
    
    nodes = []
    
    for robot_id in range(1, num_robots + 1):
        namespace = f'machine_{robot_id}'
        firefly_namespace = f'firefly_{robot_id}'
        feedback_topic = 'neural_network_feedback'

        # YOLO Detector Node
        yolo_node = Node(
            package="neural_network_detector",
            executable="yolo12_detector_node",
            name="yolo12_detector_node",
            namespace=namespace,
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'model_path': yolo_model_path,
                'labels_path': yolo_labels_path,
                'confidence_threshold': 0.45,
                'desired_class': 0,
                'publish_debug_image': True,
                'feedback_topic': feedback_topic,
            }],
            remappings=[
                ('image_raw', f'/{firefly_namespace}/xtion/rgb/image_raw'),
                ('detections', 'detections'),
                ('feedback', feedback_topic),
            ],
            output='screen',
            emulate_tty=True,
        )

        # Projector Node
        projector_node = Node(
            package='projection_model',
            executable='projection_model_node',
            name='model_distance_from_height_node',
            namespace=namespace,
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'projected_object_topic': "object_detections/projected_to_world",
                'detections_topic': "detections",
                'tracker_topic': "target_tracker/pose",
                'offset_topic': "target_tracker/offset",
                'feedback_topic': feedback_topic,
                'topics.robot': "pose/raww/std",
                'topics.camera': "camera/pose",
                'topics.optical': "camera/pose_optical",
                'camera.info_topic': "video/camera_info"
            }],
            remappings=[
                ('feedback', feedback_topic),
            ],
            output='screen',
            emulate_tty=True,
        )

        # TF from UAV Pose Node with updated parameters
        tf_node = Node(
            package='tf_from_uav_pose',
            executable='tf_from_uav_pose_node',
            name='tf_from_uav_pose',
            namespace=namespace,
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                
                # Topic names
                'poseTopicName': "pose",
                'rawPoseTopicName': "pose/raw",
                'stdPoseTopicName': "pose/corr/std",
                'stdRawPoseTopicName': "pose/raww/std",
                'throttledPoseTopicName': "throttledPose",
                'throttledUAVPoseTopicName': "throttledUAVPose",
                
                # Frame IDs
                'machineFrameID': f"machine_{robot_id}",
                'worldFrameID': "world",
                'worldENUFrameID': "world_ENU",
                'worldNWUFrameID': "world_NWU",
                'cameraFrameID': f"machine_{robot_id}_camera_link",
                'cameraRGBOpticalFrameID': f"machine_{robot_id}_camera_rgb_optical_link",
                
                # Control flags
                'dontPublishTFs': False,
                
                # Dynamic parameters
                'offsetX': 0.0,
                'offsetY': 0.0,
                'offsetZ': 0.0,
                'covarianceX': 0.0,
                'covarianceY': 0.0,
                'covarianceZ': 0.0,
                'throttleRate': 10.0,
                
                # Camera static publishing
                'cameraStaticPublish.publish': True,
                'cameraStaticPublish.topic': "camera/pose",
                'cameraStaticPublish.pose_optical_topic': "camera/pose_optical",
                'cameraStaticPublish.TFParameters': [0.18, 0.0, -0.07, 0.0, -0.38, 0.0, 0.924],
            }],
            output='screen',
            emulate_tty=True,
        )

        # Distributed KF Node
        kf_node = Node(
        package='target_tracker_distributed_kf',
        executable='distributed_kf_node',
        name='distributed_kf3d',
        namespace=namespace,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robotID': robot_id,
                'numRobots': num_robots,
                'pose_topic': 'pose',
                'measurement_topic_suffix_self': 'object_detections/projected_to_world',
                'measurement_topic_suffix': 'object_detections/projected_to_world_network',
                 'pub_topic': "target_tracker/pose",
                 'velPub_topic': "target_tracker/twist",
                 'offset_topic': "target_tracker/offset"
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO']
    )

        nodes.extend([yolo_node, projector_node, tf_node, kf_node])
    
    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output="screen"
    )
    nodes.append(rviz_node)
    
    return nodes