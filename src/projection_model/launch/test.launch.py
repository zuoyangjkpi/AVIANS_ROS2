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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import conditions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



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
    # Map to world transform
    static_tf_map_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "world"],
        output="screen"
    )
    
    # World to X3/base_link transform (this should match your drone's initial pose)
    static_tf_world_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.053302", "0", "0", "0", "world", "X3/base_link"],
        output="screen"
    )
    
    # Base_link to base_link alias (for consistency)
    static_tf_base_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.2", "0", "0", "0", "0.3491", "3.1415", "X3/base_link", "camera_link"],
        output="screen"
    )
    
    # Camera optical frame transform (standard camera convention: X-right, Y-down, Z-forward)
    # This converts from camera_link (which follows the SDF pose) to optical frame
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
    package="neural_network_detector",
    executable="yolo12_detector_node",
    name="yolo12_detector_node",
    output='screen',
    parameters=[{
        # "use_sim_time":True,
        'model_path': LaunchConfiguration('yolo_model_path'),
        'labels_path': LaunchConfiguration('yolo_labels_path'),
        'use_gpu': False,
        'confidence_threshold': 0.5,  # Will be converted to float in code
        'iou_threshold': 0.3,        # Will be converted to float in code
        'desired_class': 0,           # 0 = person class in COCO dataset
        'desired_width': 300,
        'desired_height': 300,
        'aspect_ratio': 1.333333333,      # Will be converted to float in code
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
    ],
    condition=conditions.IfCondition(LaunchConfiguration('use_person_tracking'))
    )
    

    
    # Create the projector node
    projector_node = Node(
    package='projection_model',
    executable='projection_model_node',
    name='model_distance_from_height_node',
    parameters=[{
        # 'use_sim_time': True, 
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
    # emulate_tty=True,
    arguments=['--ros-args', '--log-level', 'INFO']
)


    
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
        # gazebo,
        # gz_ros2_bridge,
        
        # Person tracking nodes (conditional)
        yolo_detector_node,  
        projector_node,      
        # Core drone nodes with delay
        # TimerAction(
        #     period=5.0,  # Reduced delay
        #     actions=[
        #         static_tf_map_world,
        #         static_tf_world_base,
        #         static_tf_base_alias,
        #         static_tf_camera,
        #         static_tf_camera_optical,
        #         controller,
        #     ]
        # ),
        
        rviz_node
    ])