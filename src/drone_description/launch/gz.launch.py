import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    models_path = os.path.join(get_package_share_directory("drone_description"), "models")

    drone_description = get_package_share_directory("drone_description")
    world_file = os.path.join(get_package_share_directory("drone_description"), "worlds", "drone_world.sdf")
    config_file = os.path.join(get_package_share_directory("drone_description"), "config", "bridge.yaml")

    # Fixed environment variables
    libgl_env = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
    gallium_env = SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe")

    # Enhanced GZ_SIM_RESOURCE_PATH to include models
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            str(Path(drone_description).parent.resolve()),
            models_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        ])
    )
    
    # Set both GAZEBO_MODEL_PATH and GZ_SIM_RESOURCE_PATH for compatibility
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r ", world_file]
                    )
                ]
             )
    
    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            os.path.join(
                get_package_share_directory("drone_description"), "lib"
            )
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{
            'config_file': config_file
        }]
    )


   # Static Transform: world → base_link (if needed)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(drone_description, "config", "drone.rviz")],  # Optional: Preload RViz config
        output="screen"
    )

    return LaunchDescription([
        libgl_env,
        gallium_env,
        set_gazebo_model_path,
        static_tf,
        gazebo_resource_path,
        gz_plugin_path,
        gazebo,
        Node(
            package='drone_description',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen'
        ),
        gz_ros2_bridge,
        rviz_node,            # Opens RViz for full visualization
    ])