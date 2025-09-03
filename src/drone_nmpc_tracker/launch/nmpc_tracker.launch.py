#!/usr/bin/env python3
"""
Launch file for NMPC Drone Person Tracker
ROS2 Jazzy compatible launch file
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for NMPC tracker"""
    
    # Package directory
    pkg_dir = FindPackageShare('drone_nmpc_tracker')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'nmpc_params.yaml']),
        description='Path to the NMPC configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    drone_namespace_arg = DeclareLaunchArgument(
        'drone_namespace',
        default_value='X3',
        description='Drone namespace'
    )
    
    # NMPC tracker node
    nmpc_tracker_node = Node(
        package='drone_nmpc_tracker',
        executable='nmpc_tracker_node',
        name='nmpc_tracker_node',
        namespace=LaunchConfiguration('drone_namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_visualization': LaunchConfiguration('enable_visualization')
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True
    )
    
    # Optional: RViz node for visualization
    def launch_rviz(context, *args, **kwargs):
        """Conditionally launch RViz based on enable_visualization parameter"""
        enable_viz = LaunchConfiguration('enable_visualization').perform(context)
        
        if enable_viz.lower() == 'true':
            # Try to find RViz config file
            try:
                rviz_config_file = PathJoinSubstitution([
                    pkg_dir, 'rviz', 'nmpc_tracker.rviz'
                ])
                
                rviz_node = Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    output='screen'
                )
                return [rviz_node]
            except:
                # Fallback: launch RViz without config file
                rviz_node = Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    output='screen'
                )
                return [rviz_node]
        else:
            return []
    
    # Optional RViz launcher
    rviz_launcher = OpaqueFunction(function=launch_rviz)
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        enable_visualization_arg,
        drone_namespace_arg,
        
        # Nodes
        nmpc_tracker_node,
        rviz_launcher,
    ])

# Alternative simple launch function for testing
def generate_simple_launch_description():
    """Generate a simple launch description for testing"""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        Node(
            package='drone_nmpc_tracker',
            executable='nmpc_tracker_node',
            name='nmpc_tracker_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'control_frequency': 10.0},
                {'person_timeout': 2.0},
                {'enable_visualization': True}
            ],
            output='screen',
            emulate_tty=True
        )
    ])

if __name__ == '__main__':
    # For testing the launch file directly
    generate_launch_description()

