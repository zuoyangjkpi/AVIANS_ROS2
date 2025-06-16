#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='Robot ID for this instance'
    )
    
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Total number of robots in the system'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Create the node
    distributed_kf_node = Node(
        package='target_tracker_distributed_kf',
        executable='distributed_kf_node',
        name='distributed_kf_3d',
        parameters=[
            {
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
                'falsePositiveThresholdSigma': 2.0,
                
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
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_id_arg,
        num_robots_arg,
        use_sim_time_arg,
        distributed_kf_node
    ])