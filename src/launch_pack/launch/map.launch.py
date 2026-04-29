import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch = LaunchDescription()
    hardware_package_path = get_package_share_directory('hardware_pkg')
    map_params_file = [os.path.join(hardware_package_path,'config','map_params.yaml')]
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': map_params_file,
        }.items()
    )

    launch.add_action(slam_launch)
    return launch