
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    world_file = os.path.join(get_package_share_directory("drone_description"), "worlds", "drone_world.sdf")
    config_file = os.path.join(get_package_share_directory("drone_description"), "config", "bridge.yaml")

    SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
    SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe"),

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
        output= 'screen',
         parameters=[{
                'config_file': config_file
            }]
    )

    return LaunchDescription([
         Node(
            package='drone_description',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen'
        ),
        gz_plugin_path,
        gazebo,
        gz_ros2_bridge,
           
    ])