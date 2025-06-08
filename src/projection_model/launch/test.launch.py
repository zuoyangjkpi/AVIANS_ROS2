import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
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
        default_value= default_yolo_model_path,
        description='Path to YOLO12 model file'
    )
    
    declare_yolo_labels_path = DeclareLaunchArgument(
        'yolo_labels_path',
        default_value= default_yolo_labels_path,
        description='Path to YOLO12 labels file'
    )

    # Paths
    drone_description = get_package_share_directory("drone_description")
    models_path = os.path.join(drone_description, "models")
    world_file = os.path.join(drone_description, "worlds", "drone_world.sdf")
    config_file = os.path.join(drone_description, "config", "bridge.yaml")
    rviz_config = os.path.join(drone_description, "config", "drone.rviz")

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

    # Static transform publishers
    static_tf_world_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )
    
    # Camera frame transform (adjust these values based on your camera mounting)
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.2", "0", "0", "0", "0.3491", "1415", "base_link", "camera_link"],
        output="screen"
    )
    
    # Camera optical frame transform (standard camera convention)
    static_tf_camera_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708", "camera_link", "camera_optical_frame"],
        output="screen"
    )

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

    # YOLO12 Detector Node
    yolo_detector_node = Node(
        package="neural_network_detector",  # Update with your actual package name
        executable="yolo12_detector_node",
        name="yolo12_detector_node",
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('yolo_model_path'),
            'labels_path': LaunchConfiguration('yolo_labels_path'),
            'use_gpu': False,
            'confidence_threshold': 0.5,
            'iou_threshold': 0.45,
            'desired_class': 0,  # Person class
            'desired_width': 640,
            'desired_height': 480,
            'aspect_ratio': 1.33333,
            'border_dropoff': 0.05,
            'publish_debug_image': True,
            'max_update_force': True,
            'max_update_rate_hz': 10.0,
            'feedback_timeout_sec': 2.0,
            'var_const_x_min': 0.1,
            'var_const_x_max': 0.1,
            'var_const_y_min': 0.1,
            'var_const_y_max': 0.1,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),  # Update with your camera topic
            ('detections', '/person_detections'),
            ('feedback', '/neural_network_feedback'),
            ('debug_image', '/detection_debug_image'),
        ],
        condition=conditions.IfCondition(LaunchConfiguration('use_person_tracking'))
    )

    # Person Tracking Projection Node
    person_tracker_node = Node(
        package="projection_model",  # Update with your actual package name
        executable="person_tracker_projection_node",
        name="person_tracker_projection_node",
        output='screen',
        parameters=[{
            'camera_frame': 'camera_optical_frame',
            'target_frame': 'world',
            'drone_frame': 'base_link',
            'assumed_person_height': 1.7,
            'min_detection_confidence': 0.6,
            'target_person_class': 0,  # Person class
            'feedback_margin_ratio': 0.15,
            'tracking_offset_distance': 3.0,
            'tracking_offset_height': 1.5,
            'publish_feedback': True,
        }],
        remappings=[
            ('detections', '/person_detections'),
            ('camera_info', '/camera/camera_info'),  # Update with your camera info topic
            ('drone_odom', '/X3/odom'),  # Update with your odometry topic
            ('target_waypoint', '/target_waypoint'),
            ('feedback', '/neural_network_feedback'),
        ],
        condition=conditions.IfCondition(LaunchConfiguration('use_person_tracking'))
    )

    # # Original YOLO node (if you want to keep it as backup)
    # yolo_node_original = Node(
    #     package="person_tracker",
    #     executable="yolo_detector",
    #     name="yolo_detector_original",
    #     output="screen",
    #     condition=conditions.UnlessCondition(LaunchConfiguration('use_person_tracking'))
    # )

    return LaunchDescription([
        # Launch arguments
        declare_use_person_tracking,
        declare_yolo_model_path,
        declare_yolo_labels_path,
        
        # Environment variables
        libgl_env,
        gallium_env,
        gazebo_resource_path,
        set_gazebo_model_path,
        gz_plugin_path,
        
        # Gazebo simulation
        gazebo,
        gz_ros2_bridge,
        
        # Person tracking nodes (conditional)
        yolo_detector_node,
        person_tracker_node,
        
        # Original YOLO node (conditional backup)
        # yolo_node_original,
        
        # Core drone nodes with delay
        TimerAction(
            period=10.0,
            actions=[
                static_tf_world_base,
                static_tf_camera,
                static_tf_camera_optical,
                controller,
                rviz_node
            ]
        )
    ])