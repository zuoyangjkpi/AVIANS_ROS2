from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('tf_from_uav_pose')
    config_dir = os.path.join(pkg_dir, 'config')
    default_config_file = os.path.join(config_dir, 'tf_from_uav_pose_params.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the ROS2 parameters file to use'
    )

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='machine_1/pose',
        description='Topic name for UAV pose input'
    )

    raw_pose_topic_arg = DeclareLaunchArgument(
        'raw_pose_topic', 
        default_value='machine_1/pose/raw',
        description='Topic name for raw UAV pose input'
    )

    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='world',
        description='World frame ID'
    )

    machine_frame_arg = DeclareLaunchArgument(
        'machine_frame',
        default_value='machine_1', 
        description='Machine/UAV base frame ID'
    )

    dont_publish_tfs_arg = DeclareLaunchArgument(
        'dont_publish_tfs',
        default_value='false',
        description='Set to true to disable TF publishing'
    )

    # Node
    tf_from_uav_pose_node = Node(
        package='tf_from_uav_pose',
        executable='tf_from_uav_pose_node',
        name='tf_from_uav_pose',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # "use_sim_time":True,
                'poseTopicName': LaunchConfiguration('pose_topic'),
                'rawPoseTopicName': LaunchConfiguration('raw_pose_topic'),
                'worldFrameID': LaunchConfiguration('world_frame'),
                'machineFrameID': LaunchConfiguration('machine_frame'),
                'dontPublishTFs': LaunchConfiguration('dont_publish_tfs'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        pose_topic_arg,
        raw_pose_topic_arg,
        world_frame_arg,
        machine_frame_arg,
        dont_publish_tfs_arg,
        tf_from_uav_pose_node
    ])